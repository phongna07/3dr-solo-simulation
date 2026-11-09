"""Measured-state per-UAV NMPC. No ROS imports and no simulated state integration.

Solver and predictive dynamics load directly from the pinned upstream submodule.
Grid planning remains a local port of motion_planning_MPC. Corridors, solver
cache, warm start and retained stopping plans belong to one vehicle's worker;
leadership, assignment, mapping and prediction exchange live in core/runtime/fleet.py.
"""

import time

import _pydecomp as pdc
import numpy as np
from ...planning.coordination import path_carrot

from .problem import SolverCache
from ...planning.paths import PathService
from ..stop_plan import StopPlan, stop_tail

# The pinned pydecomp wheel exposes its computation in _pydecomp. Import the
# binding directly: its package initializer also loads plotting libraries,
# which are unnecessary for a headless ROS controller.


class MPCVehicle:
    """One UAV's corridor MPC inside its persistent worker (VehiclePlanner)."""

    def __init__(self, name, names, cfg, grid):
        self.name, self.names, self.cfg = name, list(names), dict(cfg)
        self.warm = None
        self.stop_plan, self.active_corridor = None, None
        self.corridor_cost = None
        try:
            self.paths = PathService(self.cfg, grid)
            self.solver_cache = SolverCache(len(names) - 1, self.cfg)
            self.solver = self.solver_cache.select(4, 4)
            from .capture import SolverCapture
            SolverCapture.prepare(self.cfg, name)
        except BaseException:
            self.close()
            raise

    def payload_metrics(self):
        return {"corridor_faces": self.solver.n_faces}

    def close(self):
        from .capture import SolverCapture
        SolverCapture.close()

    def _corridor_seed(self, position, path):
        points = np.asarray(path, float)[:, :2]
        pos = np.asarray(position[:2], float)
        k = PathService.segment(points, pos)
        nxt = points[k + 1]
        if np.linalg.norm(nxt - pos) < 1e-6 and k + 2 < len(points):
            nxt = points[k + 2]
        delta = nxt - pos
        length = np.linalg.norm(delta)
        if length < 1e-6:
            delta, length = np.array([self.cfg["GRID_RESOLUTION"], 0.0]), self.cfg["GRID_RESOLUTION"]
        delta = delta * min(1.0, self.cfg["CORRIDOR_SEED_MAX"] / length)
        return np.vstack([pos, pos + delta])

    @staticmethod
    def _contains(A, b, points, margin):
        return bool(np.all(np.asarray(points).reshape(-1, 2) @ A.T <= b - margin + 1e-6))

    def _stop_tail(self, state, stamp):
        tail = stop_tail(self.stop_plan, state, stamp, self.cfg)
        if tail is None:
            self.stop_plan = None
        return tail

    def _select_corridor(self, state, stamp, candidates):
        previous = self.active_corridor
        tail = self._stop_tail(state, stamp)
        if tail is not None:
            plan, index = tail
            for A, b in candidates:
                if self._contains(A, b, plan.states[index:, :2], self.cfg["CORRIDOR_MARGIN"]):
                    return A, b
            if previous is not None:
                return previous
        for A, b in candidates:
            if self._contains(A, b, state.position[:2], 0.0):
                return A, b
        if previous is not None and self._contains(*previous, state.position[:2], 0.0):
            return previous
        if not candidates:
            raise ValueError("Corridor decomposition returned no finite polytope")
        return min(candidates, key=lambda ab: np.max(ab[0] @ state.position[:2] - ab[1]))

    @staticmethod
    def _corridor_points(scan):
        """As upstream generateSafeCorridor: hits of the current scan, no memory."""
        if scan is None:
            raise ValueError("No LiDAR scan integrated yet")
        return np.asarray(scan.endpoints, float)[np.asarray(scan.hits, bool), :2]

    def _corridor(self, state, path, stamp, scan):
        cfg = self.cfg
        position = state.position
        points = self._corridor_points(scan)
        box = cfg["CORRIDOR_BOX"]
        if not len(points):
            A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
            candidate = (A, A @ position[:2] + cfg['SENSING_RADIUS'] / np.sqrt(2))
            self.corridor_cost = candidate
            return self._select_corridor(state, stamp, [candidate])
        segment = self._corridor_seed(position, path)
        try:
            As, bs = pdc.convex_decomposition_2D(points, segment, np.array([[box, box]]))
        except Exception:
            # Upstream retains the previous static corridor if decomposition
            # fails; an absent previous corridor still fails closed below.
            As, bs = [], []
        valid = [
            (np.asarray(A), np.asarray(b).ravel())
            for A, b in zip(As, bs)
            if len(b) and np.isfinite(A).all() and np.isfinite(b).all()
        ]
        self.corridor_cost = valid[0] if valid else (None, None)
        return self._select_corridor(state, stamp, valid)

    def _fallback(self, state, stamp, reason, **metrics):
        self.warm = None
        tail = self._stop_tail(state, stamp)
        if tail is None:
            return {"ok": False, "nominal_ok": False, "reason": reason, **metrics}
        plan, index = tail
        H = self.cfg["HORIZON_LENGTH"]
        rows = plan.states[index:index + H + 1]
        rows = np.vstack([rows, np.repeat(rows[-1:], max(0, H + 1 - len(rows)), axis=0)])
        control = plan.controls[index] if index < len(plan.controls) else np.zeros(2)
        controls = plan.controls[index:index + H]
        controls = np.vstack([controls, np.zeros((H - len(controls), 2))])
        return {"ok": True, "nominal_ok": False, "position": rows[1, :2].copy(),
                "velocity": rows[1, 2:4].copy(), "acceleration": control.copy(),
                "prediction": rows.copy(), "controls": controls,
                "reason": f"stop_plan: {reason}", "retained_plan": True, **metrics}

    def solve(self, states, predictions, guidance, stamp, scan=None):
        """Local planning and solve against an immutable fleet-cycle snapshot."""
        cfg = self.cfg
        H = cfg["HORIZON_LENGTH"]
        n = self.name
        state = states[n]
        goal = guidance['goal'] if guidance['slot'] is None else guidance['slot']
        started = time.perf_counter()
        ok, path = self.paths.plan(state.position, goal)
        metrics = {"path_ms": (time.perf_counter() - started) * 1000}
        if not ok or len(path) < 2:
            return self._fallback(state, stamp, "planner_failed", **metrics)
        started = time.perf_counter()
        try:
            A, b = self._corridor(state, path, stamp, scan)
        except Exception as exc:
            metrics["corridor_ms"] = (time.perf_counter() - started) * 1000
            return self._fallback(state, stamp, f"corridor_failed: {exc}", **metrics)
        metrics["corridor_ms"] = (time.perf_counter() - started) * 1000
        started = time.perf_counter()
        try:
            cost_b = self.corridor_cost[1]
            self.solver = self.solver_cache.select(
                len(b), 0 if cost_b is None else len(cost_b))
        except Exception as exc:
            metrics["solver_setup_ms"] = (time.perf_counter() - started) * 1000
            return self._fallback(state, stamp, f"solver_setup_failed: {exc}", **metrics)
        metrics["solver_setup_ms"] = (time.perf_counter() - started) * 1000
        started = time.perf_counter()
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        X, U = self.warm if self.warm is not None else (np.tile(x0, (H + 1, 1)), np.zeros((H, 2)))
        others = [k for k in self.names if k != n]
        slot = np.zeros(2)
        if guidance['slot'] is not None:
            carrot = path_carrot(state.position, path, cfg['VMAX'] * H * cfg['TIMESTEP'])
            slot = carrot if carrot is not None else guidance['moving_slot']
        guide = len(path) > 2
        inputs = dict(
            x0=x0, A_hard=A, b_hard=b,
            A_cost=self.corridor_cost[0], b_cost=self.corridor_cost[1],
            use_corr_cost=self.corridor_cost[0] is not None,
            others=[predictions[k][:, :2] for k in others],
            nb_flags=[float(np.linalg.norm(states[k].position - state.position)
                            < cfg['SENSING_NEIGHBOR']) for k in others],
            tgt=guidance['target'], w_search=cfg['W_search_track'] if guidance['mode'] == 'SEARCH' else 0.0,
            slot=slot, w_slot=cfg['W_sat_slot'] if guidance['slot'] is not None else 0.0,
            leader=guidance['leader'], ref=path[1] if guide else np.zeros(2),
            trk_goal=guidance['goal'], s_ref=float(guide), X_init=X, U_init=U,
            w_tra=0.0 if guidance['slot'] is not None else cfg['W_tra'],
        )
        metrics['input_ms'] = (time.perf_counter() - started) * 1000
        from .capture import SolverCapture
        started = time.perf_counter()
        capture = SolverCapture(cfg, n, self.solver, stamp, inputs)
        capture_ms = (time.perf_counter() - started) * 1000
        started = time.perf_counter()
        cpu_started = time.process_time()
        ok, X, U = self.solver.solve(**inputs)
        metrics['solver_cpu_ms'] = (time.process_time() - cpu_started) * 1000
        solver_ms = (time.perf_counter() - started) * 1000
        started = time.perf_counter()
        capture.finish(ok, X, U, self.solver)
        metrics['capture_ms'] = capture_ms + (time.perf_counter() - started) * 1000
        try:
            stats = self.solver.opti.stats()
            solver_status = stats.get("return_status", "unknown")
            metrics["solver_iterations"] = stats.get("iter_count", 0)
        except RuntimeError:
            solver_status = "unavailable"
        if not ok or not np.isfinite(X).all() or not np.isfinite(U).all():
            return self._fallback(state, stamp, "mpc_failed", solver_status=solver_status,
                                  solver_ms=solver_ms, **metrics)
        self.active_corridor = (A.copy(), b.copy())
        backup = self.solver.last_backup if cfg["CONTINGENCY"] else None
        if backup is not None:
            self.stop_plan = StopPlan(backup[0].copy(), backup[1].copy(), stamp)
        elif cfg["TERMINAL_STOP"]:
            self.stop_plan = StopPlan(X.copy(), U.copy(), stamp)
        else:
            self.stop_plan = None
        self.warm = (np.vstack([X[1:], X[-1:]]), np.vstack([U[1:], U[-1:]]))
        return {
            "ok": True,
            "nominal_ok": True,
            "position": X[1, :2],
            "velocity": X[1, 2:4],
            "acceleration": U[0],
            "prediction": X,
            "controls": U,
            "reason": "tracking" if guidance["mode"] == "TRACK" else "search",
            "controller_mode": guidance["mode"],
            "solver_status": solver_status,
            "solver_ms": solver_ms,
            **metrics,
        }
