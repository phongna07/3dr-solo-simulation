"""Measured-state team NMPC. No ROS imports and no simulated state integration.

The imported solver/dynamics/grid algorithm come from motion_planning_MPC.
Team leadership, assignment, mapping, reference selection and failures live here
so they can be modified independently of transport and vehicle operation.
"""

from dataclasses import dataclass
import time

import _pydecomp as pdc
import numpy as np
from scipy.optimize import linear_sum_assignment

from .mapping import SensorGrid, warmup_scan_integration
from .mpc_problem import MPCProblem
from .planner_grid import LocalGridPlanner, warmup
from .tracking import TargetTracker

# The pinned pydecomp wheel exposes its computation in _pydecomp. Import the
# binding directly: its package initializer also loads plotting libraries,
# which are unnecessary for a headless ROS controller.


@dataclass
class VehicleState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    quaternion: np.ndarray
    stamp: float


@dataclass
class StopPlan:
    states: np.ndarray
    controls: np.ndarray
    stamp: float


class TeamController:
    def __init__(self, names, cfg, local_names=None):
        self.names, self.cfg = list(names), dict(cfg)
        self.local_names = self.names if local_names is None else list(local_names)
        if not set(self.local_names) <= set(self.names):
            raise ValueError("Local vehicles must belong to the fleet")
        self.track = TargetTracker()
        self.leader = None
        self.assignment = {}
        self.slot_scales = {n: 1.0 for n in names}
        self.planners, self.solvers, self.warm, self.paths = {}, {}, {}, {}
        self.path_goals, self.path_age = {}, {}
        self.stop_plans, self.active_corridors = {}, {}
        self.last_scans = {n: -np.inf for n in names}
        for n in self.local_names:
            planner = LocalGridPlanner(
                cfg["WORLD_BOUNDS"],
                grid_resolution=cfg["GRID_RESOLUTION"],
                inflate_radius=cfg["INFLATE_RADIUS"],
                sensing_radius=cfg["SENSING_RADIUS"],
                local_radius=cfg["LOCAL_PLAN_RADIUS"],
                unknown_policy=cfg["UNKNOWN_POLICY"],
                unknown_cost=cfg["UNKNOWN_COST"],
                goal_clamp_margin=cfg["GOAL_CLAMP_MARGIN"],
                start_snap_radius=cfg.get("START_SNAP_RADIUS", 10.0),
                search=cfg["LOCAL_SEARCH"],
            )
            planner.gmap = SensorGrid(
                cfg["WORLD_BOUNDS"],
                resolution=cfg["GRID_RESOLUTION"],
                inflate_radius=cfg["INFLATE_RADIUS"],
                sensing_radius=cfg["SENSING_RADIUS"],
                treat_unknown_as_blocked=cfg["UNKNOWN_POLICY"] == "blocked",
            )
            self.planners[n] = planner
            self.solvers[n] = MPCProblem(len(names) - 1, cfg["CORRIDOR_MAX_FACES"], cfg)
        self.predictions = {n: None for n in names}
        self.prediction_stamps = {n: -np.inf for n in names}
        if self.local_names:
            warmup()
            warmup_scan_integration()

    def _slots(self, states, seers):
        seers = [n for n in self.names if n in seers]
        if self.leader not in seers:
            self.leader = (
                min(
                    seers,
                    key=lambda n: np.linalg.norm(
                        states[n].position[:2] - self.track.x[:2]
                    ),
                )
                if seers
                else None
            )
        if self.leader is None:
            return {}
        sats = [n for n in self.names if n != self.leader]
        if not sats:
            return {}
        cells = np.asarray(self.cfg["GRID_CELLS"], float)[: len(sats)]
        if len(cells) < len(sats):
            raise ValueError("Not enough formation slots for fleet")
        lead = states[self.leader].position[:2]
        offsets = cells * self.cfg["SLOT_SPACING_RATIO"] * self.cfg["VIEWING_RADIUS"]
        costs = np.array(
            [
                [np.linalg.norm(states[n].position[:2] - lead - o) for o in offsets]
                for n in sats
            ]
        )
        rows, cols = linear_sum_assignment(costs)
        assign = {sats[r]: int(c) for r, c in zip(rows, cols)}
        if set(self.assignment) == set(sats):
            old = sum(costs[sats.index(n), i] for n, i in self.assignment.items())
            new = sum(costs[sats.index(n), i] for n, i in assign.items())
            if new > old - self.cfg["SWITCH_MARGIN"]:
                assign = self.assignment
        self.assignment = assign
        slots = {}
        for n, idx in assign.items():
            offset = offsets[idx]
            length = np.linalg.norm(offset)
            scale = 1.0
            for ratio in np.linspace(
                0.0, 1.0, max(2, int(length / self.cfg["GRID_RESOLUTION"]))
            ):
                point = lead + ratio * offset
                blocked = any(
                    p.gmap.is_blocked(*p.gmap.world_to_grid(*point))
                    for p in self.planners.values()
                )
                if blocked:
                    scale = max(self.cfg["SLOT_MIN_RATIO"], ratio - 0.1)
                    break
            scale = min(
                scale,
                self.slot_scales[n]
                + self.cfg["SLOT_EXPAND_SPEED"]
                * self.cfg["TIMESTEP"]
                / max(length, 0.1),
            )
            self.slot_scales[n] = scale
            slots[n] = lead + offset * scale
        return slots

    def _path(self, name, position, goal):
        planner = self.planners[name]
        previous = self.paths.get(name)
        refresh = (
            previous is None or len(previous) < 2
            or self.path_age.get(name, 0) >= self.cfg["MAX_COMMIT_AGE"]
        )
        refresh |= (
            name not in self.path_goals
            or np.linalg.norm(goal - self.path_goals.get(name, goal))
            > self.cfg["TARGET_REPLAN_THRESHOLD"]
        )
        if not refresh:
            points = np.asarray(previous, float)
            nearest = self._path_segment(points, position[:2])
            if nearest + 2 < len(points) and np.linalg.norm(points[nearest + 1] - position[:2]) < 1e-6:
                nearest += 1
            trimmed = np.vstack(
                [position[:2], points[min(nearest + 1, len(points) - 1) :]]
            )
            refresh = not all(
                planner.gmap.segment_clear(a, b)
                for a, b in zip(trimmed[:-1], trimmed[1:])
            )
            if not refresh:
                self.path_age[name] += 1
                return True, trimmed.tolist()
        planner.initialize(position, goal)
        ok, path, *_ = planner.plan(
            np.empty((0, 2)), self.cfg["ROBOT_RADIUS"], observe=False
        )
        if ok:
            self.path_goals[name] = np.asarray(goal).copy()
            self.path_age[name] = 0
        return ok, path

    @staticmethod
    def _path_segment(points, position):
        """Project onto segments so trimming a committed path cannot skip a turn."""
        delta = points[1:] - points[:-1]
        along = np.clip(np.sum((position - points[:-1]) * delta, axis=1)
                        / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0, 1)
        return int(np.argmin(np.linalg.norm(points[:-1] + along[:, None] * delta
                                           - position, axis=1)))

    def _corridor_seed(self, position, path):
        points = np.asarray(path, float)[:, :2]
        pos = np.asarray(position[:2], float)
        k = self._path_segment(points, pos)
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

    def _stop_tail(self, name, state, stamp):
        """Use simulation time and measured-state agreement, never a solve counter.

        A model certificate cannot justify replay after the FCU deviates from it.
        The publisher's existing expiry/hold behavior handles a rejected tail.
        """
        plan = self.stop_plans.get(name)
        if plan is None:
            return None
        elapsed = (stamp - plan.stamp) / self.cfg["TIMESTEP"]
        if not np.isfinite(elapsed) or elapsed < -1e-6:
            self.stop_plans.pop(name, None)
            return None
        index = min(int(np.floor(elapsed + 1e-6)), len(plan.states) - 1)
        measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
        for values, expected, key in zip(
            measured, np.split(plan.states[index], 3),
            ("BACKUP_POSITION_TOLERANCE", "BACKUP_VELOCITY_TOLERANCE", "BACKUP_ACCEL_TOLERANCE"),
        ):
            if not np.isfinite(values).all() or np.linalg.norm(values - expected) > self.cfg[key]:
                self.stop_plans.pop(name, None)
                return None
        return plan, index

    def _select_corridor(self, name, state, stamp, candidates):
        previous = self.active_corridors.get(name)
        tail = self._stop_tail(name, state, stamp)
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

    def _corridor(self, name, state, path, stamp):
        cfg = self.cfg
        position = state.position
        points = self.planners[name].gmap.surface_points(
            position, cfg["SENSING_RADIUS"] + 1.0
        )
        box = cfg["CORRIDOR_BOX"]
        if not len(points):
            A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
            return self._select_corridor(name, state, stamp, [
                (A, A @ position[:2] + cfg["SENSING_RADIUS"] / np.sqrt(2))])
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
        return self._select_corridor(name, state, stamp, valid)

    def _fallback(self, name, state, stamp, reason, **metrics):
        self.warm.pop(name, None)
        tail = self._stop_tail(name, state, stamp)
        if tail is None:
            return {"ok": False, "nominal_ok": False, "reason": reason, **metrics}
        plan, index = tail
        H = self.cfg["HORIZON_LENGTH"]
        rows = plan.states[index:index + H + 1]
        rows = np.vstack([rows, np.repeat(rows[-1:], max(0, H + 1 - len(rows)), axis=0)])
        control = plan.controls[index] if index < len(plan.controls) else np.zeros(2)
        self.predictions[name] = rows[:, :2].copy()
        self.prediction_stamps[name] = stamp
        return {"ok": True, "nominal_ok": False, "position": rows[1, :2].copy(),
                "velocity": rows[1, 2:4].copy(), "acceleration": control.copy(),
                "prediction": rows.copy(), "reason": f"stop_plan: {reason}", **metrics}

    def integrate_scans(self, scans):
        """Update only maps owned by this instance."""
        for n in self.local_names:
            if scans[n].stamp > self.last_scans[n]:
                self.planners[n].gmap.integrate(scans[n])
                self.last_scans[n] = scans[n].stamp

    def neighbor_predictions(self, states, stamp):
        """Freeze neighbor predictions before any vehicle solve updates them."""
        H, dt = self.cfg["HORIZON_LENGTH"], self.cfg["TIMESTEP"]
        predictions = {}
        for n, s in states.items():
            old = self.predictions[n]
            if old is None or stamp - self.prediction_stamps[n] > 3 * dt:
                predictions[n] = (
                    s.position[:2] + np.arange(H + 1)[:, None] * dt * s.velocity[:2]
                )
            else:
                shifted = np.vstack([old[1:], old[-1:]])
                # Predictions describe current measured states, even when the
                # FCU response differs from the nominal acceleration-lag model.
                predictions[n] = shifted + (s.position[:2] - shifted[0])
        return predictions

    def step(self, states, scans, stamp, seers):
        cfg = self.cfg
        target = self.track.trajectory(
            stamp, cfg["HORIZON_LENGTH"], cfg["TIMESTEP"],
            cfg.get("TARGET_MAX_SPEED", cfg["VMAX"]),
        )
        self.integrate_scans(scans)
        slots = self._slots(states, seers)
        predictions = self.neighbor_predictions(states, stamp)
        return {
            n: self.solve_vehicle(n, states, predictions, target, slots, stamp)
            for n in self.local_names
        }

    def solve_vehicle(self, n, states, predictions, target, slots, stamp):
        """Local planning and solve against an immutable fleet-cycle snapshot."""
        cfg = self.cfg
        H = cfg["HORIZON_LENGTH"]
        state = states[n]
        goal = slots.get(n, target[-1])
        started = time.perf_counter()
        ok, path = self._path(n, state.position, goal)
        metrics = {"path_ms": (time.perf_counter() - started) * 1000}
        self.paths[n] = path
        if not ok or len(path) < 2:
            return self._fallback(n, state, stamp, "planner_failed", **metrics)
        started = time.perf_counter()
        try:
            A, b = self._corridor(n, state, path, stamp)
            if len(b) > self.solvers[n].n_faces:
                self.solvers[n] = MPCProblem(len(self.names) - 1, len(b) + 4, cfg)
                self.warm.pop(n, None)
        except Exception as exc:
            return self._fallback(n, state, stamp, f"corridor_failed: {exc}", **metrics)
        metrics["corridor_ms"] = (time.perf_counter() - started) * 1000
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        X, U = self.warm.get(n, (np.tile(x0, (H + 1, 1)), np.zeros((H, 2))))
        others = [k for k in self.names if k != n]
        # Guide satellites along the path; switch to the moving slot nearby.
        guide = len(path) > 2
        slot = np.asarray(path[1]) if guide else slots.get(n, np.zeros(2))
        ok, X, U = self.solvers[n].solve(
            x0=x0,
            A_hard=A,
            b_hard=b,
            A_cost=A,
            b_cost=b,
            use_corr_cost=True,
            others=[predictions[k] for k in others],
            nb_flags=[
                float(
                    np.linalg.norm(states[k].position - state.position)
                    < cfg["SENSING_NEIGHBOR"]
                )
                for k in others
            ],
            tgt=target,
            w_search=cfg["W_search_track"] if self.leader is None else 0.0,
            slot=slot,
            w_slot=cfg["W_sat_slot"] if n in slots else 0.0,
            leader=n == self.leader,
            ref=path[1],
            trk_goal=target[-1],
            s_ref=float(guide),
            X_init=X,
            U_init=U,
            w_tra=0.0 if n in slots else cfg["W_tra"],
        )
        try:
            stats = self.solvers[n].opti.stats()
            solver_status = stats.get("return_status", "unknown")
            metrics["solver_iterations"] = stats.get("iter_count", 0)
        except RuntimeError:
            solver_status = "unavailable"
        solver_ms = self.solvers[n].solve_times[-1] * 1000
        if not ok or not np.isfinite(X).all() or not np.isfinite(U).all():
            return self._fallback(n, state, stamp, "mpc_failed", solver_status=solver_status,
                                  solver_ms=solver_ms, **metrics)
        self.active_corridors[n] = (A.copy(), b.copy())
        backup = self.solvers[n].last_backup if cfg["CONTINGENCY"] else None
        if backup is not None:
            self.stop_plans[n] = StopPlan(backup[0].copy(), backup[1].copy(), stamp)
        elif cfg["TERMINAL_STOP"]:
            self.stop_plans[n] = StopPlan(X.copy(), U.copy(), stamp)
        else:
            self.stop_plans.pop(n, None)
        self.predictions[n] = X[:, :2].copy()
        self.prediction_stamps[n] = stamp
        self.warm[n] = (np.vstack([X[1:], X[-1:]]), np.vstack([U[1:], U[-1:]]))
        return {
            "ok": True,
            "nominal_ok": True,
            "position": X[1, :2],
            "velocity": X[1, 2:4],
            "acceleration": U[0],
            "prediction": X,
            "reason": "tracking" if self.leader else "search",
            "solver_status": solver_status,
            "solver_ms": solver_ms,
            **metrics,
        }
