"""Measured-state team NMPC. No ROS imports and no simulated state integration.

The imported solver/dynamics/grid algorithm come from motion_planning_MPC.
Team leadership, assignment, mapping, reference selection and failures live here
so they can be modified independently of transport and vehicle operation.
"""

from dataclasses import dataclass

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


class TeamController:
    def __init__(self, names, cfg):
        self.names, self.cfg = list(names), dict(cfg)
        self.track = TargetTracker()
        self.leader = None
        self.assignment = {}
        self.slot_scales = {n: 1.0 for n in names}
        self.planners, self.solvers, self.warm, self.paths = {}, {}, {}, {}
        self.path_goals, self.path_age = {}, {}
        self.last_scans = {n: -np.inf for n in names}
        for n in names:
            planner = LocalGridPlanner(
                cfg["WORLD_BOUNDS"],
                grid_resolution=cfg["GRID_RESOLUTION"],
                inflate_radius=cfg["INFLATE_RADIUS"],
                sensing_radius=cfg["SENSING_RADIUS"],
                local_radius=cfg["LOCAL_PLAN_RADIUS"],
                unknown_policy=cfg["UNKNOWN_POLICY"],
                unknown_cost=cfg["UNKNOWN_COST"],
                goal_clamp_margin=cfg["GOAL_CLAMP_MARGIN"],
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
        warmup()
        warmup_scan_integration()

    def _slots(self, states, seers):
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
            previous is None or self.path_age.get(name, 0) >= self.cfg["MAX_COMMIT_AGE"]
        )
        refresh |= (
            name not in self.path_goals
            or np.linalg.norm(goal - self.path_goals.get(name, goal))
            > self.cfg["TARGET_REPLAN_THRESHOLD"]
        )
        if not refresh:
            points = np.asarray(previous, float)
            nearest = int(np.argmin(np.linalg.norm(points - position[:2], axis=1)))
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

    def _corridor(self, name, position, path):
        cfg = self.cfg
        points = self.planners[name].gmap.surface_points(
            position, cfg["SENSING_RADIUS"] + 1.0
        )
        box = min(cfg["CORRIDOR_BOX"], cfg["SENSING_RADIUS"] / np.sqrt(2))
        if not len(points):
            A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
            return A, A @ position[:2] + box
        segment = np.asarray(path[:2], float)
        if np.linalg.norm(segment[1] - segment[0]) < 1e-6:
            segment[1] = segment[0] + [0.05, 0.0]
        As, bs = pdc.convex_decomposition_2D(points, segment, np.array([[box, box]]))
        valid = [
            (np.asarray(A), np.asarray(b).ravel())
            for A, b in zip(As, bs)
            if np.isfinite(A).all() and np.isfinite(b).all()
        ]
        if not valid:
            raise ValueError("Corridor decomposition returned no finite polytope")
        return min(valid, key=lambda ab: np.max(ab[0] @ position[:2] - ab[1]))

    def step(self, states, scans, stamp, seers):
        cfg = self.cfg
        H = cfg["HORIZON_LENGTH"]
        dt = cfg["TIMESTEP"]
        target = self.track.trajectory(stamp, H, dt, cfg["VMAX"])
        for n in self.names:
            if scans[n].stamp > self.last_scans[n]:
                self.planners[n].gmap.integrate(scans[n])
                self.last_scans[n] = scans[n].stamp
        slots = self._slots(states, seers)
        # Freeze neighbor predictions for the entire solve cycle.
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
        results = {}
        for n in self.names:
            state = states[n]
            goal = slots.get(n, target[-1])
            ok, path = self._path(n, state.position, goal)
            self.paths[n] = path
            if not ok or len(path) < 2:
                results[n] = {"ok": False, "reason": "planner_failed"}
                continue
            try:
                A, b = self._corridor(n, state.position, path)
                if len(b) > self.solvers[n].n_faces:
                    self.solvers[n] = MPCProblem(len(self.names) - 1, len(b) + 4, cfg)
                    self.warm.pop(n, None)
            except Exception as exc:
                results[n] = {"ok": False, "reason": f"corridor_failed: {exc}"}
                continue
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
                solver_status = (
                    self.solvers[n].opti.stats().get("return_status", "unknown")
                )
            except RuntimeError:
                solver_status = "unavailable"
            solver_ms = self.solvers[n].solve_times[-1] * 1000
            if not ok or not np.isfinite(X).all() or not np.isfinite(U).all():
                results[n] = {
                    "ok": False,
                    "reason": "mpc_failed",
                    "solver_status": solver_status,
                    "solver_ms": solver_ms,
                }
                continue
            self.predictions[n] = X[:, :2].copy()
            self.prediction_stamps[n] = stamp
            self.warm[n] = (np.vstack([X[1:], X[-1:]]), np.vstack([U[1:], U[-1:]]))
            results[n] = {
                "ok": True,
                "position": X[1, :2],
                "velocity": X[1, 2:4],
                "acceleration": U[0],
                "prediction": X,
                "reason": "tracking" if self.leader else "search",
                "solver_status": solver_status,
                "solver_ms": solver_ms,
            }
        return results
