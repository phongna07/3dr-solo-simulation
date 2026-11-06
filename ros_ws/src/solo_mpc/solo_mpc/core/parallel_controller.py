"""One persistent spawned process per UAV, with coherent fleet-cycle snapshots.

Workers own maps, planners, corridors, solvers and warm starts. Only formation
assignment and prediction exchange remain in the coordinator. No ROS objects or
CasADi objects cross a process boundary. Spawn avoids inheriting DDS/native state.
"""

import faulthandler
import math
import os
from pathlib import Path
import signal
import time
import threading
from types import SimpleNamespace

from .controller import TeamController
from .workers import WorkerPool


class MapSnapshot:
    """Read-only inflated map for the existing formation-slot checks."""

    def __init__(self, cfg):
        self.ox, self.oy = cfg["WORLD_BOUNDS"][:2]
        self.resolution = cfg["GRID_RESOLUTION"]
        self.blocked = None
        self.occupied = None
        self.unknown = None

    def world_to_grid(self, wx, wy):
        return int(math.floor((wx - self.ox) / self.resolution)), int(math.floor((wy - self.oy) / self.resolution))

    def is_blocked(self, ix, iy):
        if self.blocked is None:
            raise RuntimeError("Formation map has not been received")
        height, width = self.blocked.shape
        return not (0 <= ix < width and 0 <= iy < height) or bool(self.blocked[iy, ix])


def vehicle_worker(connection, name, names, cfg):
    """Process entry point; the fleet name list retains all neighbor constraints."""
    trace = None
    try:
        if cfg.get('DIAGNOSTIC_DIR'):
            trace = (Path(cfg['DIAGNOSTIC_DIR']).parent / 'logs' /
                     f'worker-{name}.traceback.log').open('a')
            faulthandler.register(signal.SIGUSR1, file=trace, all_threads=True)
        controller = TeamController(names, cfg, local_names=[name])
        grid = controller.planners[name].gmap
        connection.send({"phase": "ready", "pid": os.getpid(), "vehicle": name,
                         "finished_wall": time.perf_counter()})
        while True:
            job = connection.recv()
            phase, cycle = job["phase"], job["cycle"]
            if phase == "stop":
                return
            started = time.perf_counter()
            try:
                if phase == "map":
                    scan = job["scan"]
                    updated = scan.stamp > controller.last_scans[name] and bool(len(scan.endpoints))
                    controller.integrate_scans({name: scan})
                    # Inflation belongs to the worker as well as ray integration.
                    grid._ensure()
                    payload = {"blocked": grid.blocked, "occupied": grid.L >= grid.occ_logit,
                               "unknown": grid.unknown, "scan_stamp": scan.stamp,
                               "scan_usable_rays": len(scan.endpoints),
                               "scan_hits": int(scan.hits.sum()), "map_updated": updated}
                elif phase == "solve":
                    payload = {"result": controller.solve_vehicle(
                        name, job["states"], job["predictions"], job["guidance"], job["stamp"],
                    ), "n_faces": controller.solvers[name].n_faces}
                else:
                    raise ValueError(f"Unknown worker phase: {phase}")
                connection.send({"phase": phase, "cycle": cycle, "pid": os.getpid(),
                                 "started_wall": started, "finished_wall": time.perf_counter(),
                                 **payload})
            except Exception as exc:
                connection.send({"phase": phase, "cycle": cycle, "pid": os.getpid(),
                                 "finished_wall": time.perf_counter(),
                                 "error": f"{type(exc).__name__}: {exc}"})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({"phase": "ready", "finished_wall": time.perf_counter(),
                             "error": f"{type(exc).__name__}: {exc}"})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        from .diagnostics import SolverCapture
        SolverCapture.close()
        if trace is not None:
            faulthandler.unregister(signal.SIGUSR1)
            trace.close()
        connection.close()


class ParallelTeamController(TeamController):
    """Same measured-state interface as TeamController; local work is concurrent."""

    def __init__(self, names, cfg, timeout=1.0, startup_timeout=30.0, sequential=False):
        super().__init__(names, cfg, local_names=[])
        self.sequential = sequential
        self.vehicle_metrics = {}
        self.cycle = 0
        self.closed = False
        self._cycle_lock = threading.Lock()
        self.planners = {n: SimpleNamespace(gmap=MapSnapshot(cfg)) for n in names}
        self.pool = WorkerPool(names, vehicle_worker, (self.names, self.cfg),
                               timeout, startup_timeout, on_timeout=self._trace_timeout,
                               process_prefix="mpc")

    def _trace_timeout(self, worker):
        # MPC diagnostic policy stays outside reusable process orchestration.
        if self.cfg.get('DIAGNOSTIC_DIR') and worker.process.is_alive():
            try:
                os.kill(worker.process.pid, signal.SIGUSR1)
                worker.process.join(timeout=0.05)
            except ProcessLookupError:
                pass

    def step(self, states, scans, stamp, seers, tracker=None):
        if not self._cycle_lock.acquire(blocking=False):
            raise RuntimeError('A fleet cycle is already in progress')
        try:
            return self._step(states, scans, stamp, seers, tracker)
        finally:
            self._cycle_lock.release()

    def _step(self, states, scans, stamp, seers, tracker=None):
        if self.closed:
            raise RuntimeError("Parallel controller is closed")
        self.cycle += 1
        self.pool.cycle = self.cycle
        self.vehicle_metrics = {}
        # Capture predictions before dispatch; no within-cycle solve feeds another.
        predictions = self.neighbor_predictions(states, stamp)
        mapped = self.pool.dispatch("map", {n: {"scan": scans[n]} for n in self.names})
        if any("error" in m for m in mapped.values()):
            # Assignment requires all maps. Do not use a missing/stale map as free.
            return {n: {"ok": False, "reason": mapped[n].get("error", "fleet_mapping_failed")}
                    for n in self.names}
        for n, message in mapped.items():
            for key in ('blocked', 'occupied', 'unknown'):
                setattr(self.planners[n].gmap, key, message[key])
        guidance = self.prepare_guidance(states, predictions, stamp, seers, tracker)
        solved = self.pool.dispatch("solve", {n: dict(states=states, predictions=predictions,
                                                 guidance=guidance[n], stamp=stamp) for n in self.names},
                                    ordered=self.sequential, deadline_error="fleet_solve_deadline")
        results = {}
        for n, message in solved.items():
            if "error" in message:
                results[n] = {"ok": False, "reason": message["error"]}
                continue
            result = message["result"]
            self.vehicle_metrics[n] = {
                "worker_pid": message["pid"], "cycle": self.cycle,
                **{key: mapped[n][key] for key in (
                    "scan_stamp", "scan_usable_rays", "scan_hits", "map_updated")},
                "mapping_ms": (mapped[n]["finished_wall"]-mapped[n]["started_wall"])*1000,
                "planning_and_solve_ms": (message["finished_wall"]-message["started_wall"])*1000,
                "map_started_wall": mapped[n]["started_wall"],
                "map_finished_wall": mapped[n]["finished_wall"],
                "solve_started_wall": message["started_wall"],
                "solve_finished_wall": message["finished_wall"],
                "corridor_faces": message["n_faces"],
            }
            result.update(self.vehicle_metrics[n])
            results[n] = result
            if result["ok"]:
                self.predictions[n] = result["prediction"].copy()
                self.prediction_stamps[n] = stamp
        return {n: results[n] for n in self.names}

    def close(self):
        """Wait for the bounded active cycle before closing its pipes."""
        with self._cycle_lock:
            self._close()

    def _close(self):
        if self.closed:
            return
        self.closed = True
        self.pool.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
