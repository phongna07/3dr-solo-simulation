"""One persistent spawned process per UAV, with coherent fleet-cycle snapshots.

Workers own maps and a vehicle planner (solvers, warm starts, paths). Only
leadership/formation assignment and prediction exchange remain in the
coordinator. No ROS objects or CasADi objects cross a process boundary. Spawn
avoids inheriting DDS/native state. Algorithms differ only in the importable
planner class each worker builds and in the guidance goal policy.
"""

import faulthandler
import importlib
import math
import os
from pathlib import Path
import signal
import time
import threading

import numpy as np

from .algorithm import (CycleInput, CycleResult, ExecutableTrajectory,
                        NeighborPrediction, VehicleResult, VehicleState)
from .coordination import GOAL_POLICIES, FleetGuidance, guidance_goal
from .tracking import TargetTracker
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


def planner_class(spec):
    """'package.module:Class' strings stay picklable for spawned workers."""
    module, _, name = spec.partition(':')
    return getattr(importlib.import_module(module), name)


def fleet_worker(connection, name, names, cfg, planner_spec):
    """Process entry point; the fleet name list retains all neighbor constraints."""
    trace = planner = None
    try:
        if cfg.get('DIAGNOSTIC_DIR'):
            trace = (Path(cfg['DIAGNOSTIC_DIR']).parent / 'logs' /
                     f'worker-{name}.traceback.log').open('a')
            faulthandler.register(signal.SIGUSR1, file=trace, all_threads=True)
        from .mapping import SensorGrid, warmup_scan_integration
        grid = SensorGrid(
            cfg["WORLD_BOUNDS"],
            resolution=cfg["GRID_RESOLUTION"],
            inflate_radius=cfg["INFLATE_RADIUS"],
            sensing_radius=cfg["SENSING_RADIUS"],
            treat_unknown_as_blocked=cfg["UNKNOWN_POLICY"] == "blocked",
        )
        warmup_scan_integration()
        last_scan, scan = -np.inf, None
        planner = planner_class(planner_spec)(name, names, cfg, grid)
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
                    received = job["scan"]
                    updated = received.stamp > last_scan and bool(len(received.endpoints))
                    if received.stamp > last_scan:
                        grid.integrate(received)
                        last_scan, scan = received.stamp, received
                    # Inflation belongs to the worker as well as ray integration.
                    grid._ensure()
                    payload = {"blocked": grid.blocked, "occupied": grid.L >= grid.occ_logit,
                               "unknown": grid.unknown, "scan_stamp": received.stamp,
                               "scan_usable_rays": len(received.endpoints),
                               "scan_hits": int(received.hits.sum()), "map_updated": updated}
                elif phase == "solve":
                    payload = {"result": planner.solve(
                        job["states"], job["predictions"], job["guidance"], job["stamp"], scan,
                    ), "metrics": planner.payload_metrics()}
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
        if planner is not None:
            planner.close()
        if trace is not None:
            faulthandler.unregister(signal.SIGUSR1)
            trace.close()
        connection.close()


class FleetCoordinator(FleetGuidance):
    """Leadership, assignment, guidance and prediction exchange of the fleet."""

    def __init__(self, names, cfg, goal_policy='pursuit'):
        super().__init__(names, dict(cfg))
        if goal_policy not in GOAL_POLICIES:
            raise ValueError(f'Unknown guidance goal policy: {goal_policy}')
        self.goal_policy = goal_policy
        self.track = TargetTracker(cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
        self.guidance = {}
        self.maps = {n: MapSnapshot(cfg) for n in names}
        self.predictions = {n: None for n in names}
        self.prediction_stamps = {n: -np.inf for n in names}

    def prepare_guidance(self, states, predictions, stamp, seers, tracker=None):
        track = self.track if tracker is None else tracker
        slots = self.coordinate(states, predictions, track.position(stamp), seers,
                                list(self.maps.values()))
        target = track.trajectory(stamp, self.cfg['HORIZON_LENGTH'],
                                  self.cfg['TIMESTEP'], 2 * self.cfg['VMAX'])
        self.guidance = {
            n: dict(mode=self.modes[n], leader=n == self.leader, target=target,
                    goal=guidance_goal(self.goal_policy, states[n].position,
                                       track.pursuit(states[n].position, stamp, self.cfg),
                                       n == self.leader, self.cfg),
                    slot=slots.get(n), moving_slot=self.moving_slot(n, predictions) if n in slots else None)
            for n in self.names
        }
        return self.guidance

    def neighbor_predictions(self, states, stamp):
        """Age-align cached plans to measured state, frozen before any solve."""
        H, dt = self.cfg['HORIZON_LENGTH'], self.cfg['TIMESTEP']
        predictions = {}
        for n, state in states.items():
            old = self.predictions[n]
            age = stamp - self.prediction_stamps[n]
            if old is None or age < 0 or age > 3 * dt:
                rows = np.tile(np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]], (H + 1, 1))
                rows[:, :2] += np.arange(H + 1)[:, None] * dt * state.velocity[:2]
            else:
                steps = min(H, max(0, int(np.floor(age / dt + 1e-6))))
                rows = np.vstack([old[steps:], np.repeat(old[-1:], steps, axis=0)])
                rows = rows.copy()
                rows[:, :2] += state.position[:2] - rows[0, :2]
                rows[0] = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
            predictions[n] = rows
        return predictions


class ParallelFleet(FleetCoordinator):
    """Map barrier, guidance, per-UAV solve dispatch and prediction commit."""

    def __init__(self, names, cfg, planner_spec, goal_policy='pursuit', timeout=1.0,
                 startup_timeout=30.0, sequential=False, process_prefix='mpc'):
        super().__init__(names, cfg, goal_policy)
        self.sequential = sequential
        self.vehicle_metrics = {}
        self.cycle = 0
        self.closed = False
        self._cycle_lock = threading.Lock()
        self.pool = WorkerPool(names, fleet_worker, (self.names, self.cfg, planner_spec),
                               timeout, startup_timeout, on_timeout=self._trace_timeout,
                               process_prefix=process_prefix)

    def _trace_timeout(self, worker):
        # Diagnostic policy stays outside reusable process orchestration.
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
                setattr(self.maps[n], key, message[key])
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
                **message["metrics"],
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


class FleetAlgorithm:
    """Algorithm over ParallelFleet: worker results become runtime contracts.

    A result's ``prediction`` rows are what teammates plan against; its
    ``trajectory`` rows (default: the prediction) and ``controls`` are executed.
    """

    NAME = None
    PLANNER = None
    GOAL_POLICY = 'pursuit'
    UPSTREAM_MODULES = ()

    def __init__(self, names, cfg, settings):
        self.controller = ParallelFleet(
            names, cfg, self.PLANNER, self.GOAL_POLICY,
            timeout=settings.get('controller_worker_timeout_wall', 1.0),
            startup_timeout=settings.get('controller_worker_startup_timeout_wall', 30.0),
            sequential=settings.get('controller_execution', 'parallel') == 'sequential',
        )

    @property
    def track(self):
        return self.controller.track

    @track.setter
    def track(self, tracker):
        self.controller.track = tracker

    def step(self, inputs: CycleInput) -> CycleResult:
        raw = self.controller.step(inputs.states, inputs.scans, inputs.source_stamp,
                                   inputs.seers, tracker=inputs.tracker)
        dt = self.controller.cfg['TIMESTEP']
        vehicles = {}
        for name, out in raw.items():
            rows, controls = out.get('prediction'), out.get('controls')
            executed = out.get('trajectory', rows)
            prediction = (None if rows is None else
                          NeighborPrediction(inputs.source_stamp, dt, rows))
            trajectory = (None if executed is None or controls is None else
                          ExecutableTrajectory(inputs.source_stamp, dt,
                                               executed[:, :2], executed[:, 2:4], controls))
            ok = out['ok']
            nominal = out.get('nominal_ok', ok)
            vehicles[name] = VehicleResult(
                nominal_success=nominal, fallback_used=ok and not nominal,
                publication_eligible=ok, reason=out['reason'],
                trajectory=trajectory, prediction=prediction,
                diagnostics={key: value for key, value in out.items() if key not in (
                    'ok', 'nominal_ok', 'reason', 'prediction', 'controls', 'trajectory',
                    'position', 'velocity', 'acceleration')})
        return CycleResult(inputs.source_stamp, vehicles, self.diagnostics())

    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult:
        return result

    def occupancy_maps(self):
        return [grid for grid in self.controller.maps.values()
                if grid.occupied is not None and grid.unknown is not None]

    def diagnostics(self):
        c = self.controller
        return dict(algorithm=self.NAME, leader=c.leader, controller_modes=dict(c.modes),
                    assignment=dict(c.assignment), slot_scales=dict(c.slot_scales))

    def close(self):
        self.controller.close()
