"""Baseline ``impc_ob_rec``: upstream impc_ob.py IMPC-OB with recovery per UAV.

Upstream IMPCOB (recover=True) runs unchanged on the shared leader/slot layers
with this worker's committed local path as reference path. Its QCQP is built
once at upstream's maximum of 24 faces and solved once at hover before
readiness, so no solve pays a rebuild or first-solve cost. Obstacles are the
hits of the latest integrated LiDAR scan; teammates enter as upstream robots in
fleet order, and their deadlock state (priority, distance, contest) arrives
from the previous cycle through the fleet broadcast channel.

The predetermined trajectory is this vehicle's previously executed plan,
shifted by the elapsed simulation time (upstream shifts one row per step). The
full solved state is the prediction: upstream keeps only the shifted warm start
``_X``, so its first row is replaced by the measured initial state, which the
solve constrains it to equal. A failed step (corridor, solve or path) ignores
upstream's counter-indexed command: it replays the stop plan by simulation time
while the measured state agrees and upstream ``_tail_collides`` accepts it, else
brakes with upstream ``_brake_traj``; both are visible fallbacks.
"""

import time
from types import SimpleNamespace

import numpy as np

from ... import upstream
from ...planning.paths import PathService
from ..command_rollout import brake_extend
from ..speed_recovery import enable_speed_recovery, set_speed_limit
from ..stop_plan import StopPlan, stop_tail
from ..upstream_robot import UpstreamRobot, neighbor_robots, to_upstream


class IMPCOBRecVehicle:
    def __init__(self, name, names, cfg, grid):
        self.name, self.names, self.cfg = name, list(names), dict(cfg)
        self.robot = UpstreamRobot(self.names.index(name))
        self.paths = PathService(self.cfg, grid)
        module = upstream.load('impc_ob')
        n_others = len(self.names) - 1
        self.impc = module.IMPCOB(self.robot, n_others, self.cfg, recover=True)
        self.impc.n_faces = self.impc.max_faces
        self.impc.prob = module.IMPCOBProblem(n_others, self.impc.max_faces, self.cfg)
        prob = self.impc.prob
        enable_speed_recovery(prob.opti, prob.p, prob.X, self.cfg["HORIZON_LENGTH"], self.cfg)
        self.warmup_ok = self._warmup(n_others)
        self.own_plan = None
        self.stop_plan = None

    def _warmup(self, n_others):
        """One hover solve without obstacles or active teammates; counters reset."""
        H, prob = self.cfg["HORIZON_LENGTH"], self.impc.prob
        x0 = np.zeros(6)
        set_speed_limit(prob.opti, prob.p, x0, self.cfg, H)
        ok, *_ = prob.solve(x0, np.array([1.0, 0.0]), np.full(H, 10.0),
                            [(np.zeros((0, 2)), np.zeros(0))] * H,
                            [np.zeros((H, 2))] * n_others, [np.zeros(H)] * n_others,
                            np.zeros(n_others), np.ones(n_others),
                            np.tile(x0, (H + 1, 1)), np.zeros((H, 2)))
        prob.solve_times, prob.n_fail = [], 0
        return bool(ok)

    def _predetermined(self, x0, stamp):
        """Previous executed plan aligned to ``stamp``; at rest here if none is current."""
        H, dt = self.cfg["HORIZON_LENGTH"], self.cfg["TIMESTEP"]
        if self.own_plan is not None:
            rows, planned = self.own_plan
            steps = int(np.floor((stamp - planned) / dt + 1e-6))
            if 0 <= steps <= H:
                return np.vstack([rows[steps:], np.repeat(rows[-1:], steps, axis=0)])
        return np.tile(x0, (H + 1, 1))

    def solve(self, states, predictions, guidance, stamp, scan):
        cfg, impc = self.cfg, self.impc
        H = cfg["HORIZON_LENGTH"]
        state = states[self.name]
        goal = np.asarray(guidance['goal'] if guidance['slot'] is None else guidance['slot'], float)[:2]
        obstacles = (np.empty((0, 2)) if scan is None
                     else np.asarray(scan.endpoints, float)[np.asarray(scan.hits, bool), :2])
        robot = self.robot.measure(state)
        z = robot.state[2]
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        set_speed_limit(impc.prob.opti, impc.prob.p, x0, cfg, H)
        robot.states_prediction = to_upstream(self._predetermined(x0, stamp), z)
        peers = guidance.get('peers', {})
        others = neighbor_robots(self.names, self.name, states, predictions)
        team = [impc]
        for other, robot_other in zip([n for n in self.names if n != self.name], others):
            shared = peers.get(other) or {}
            robot_other.impc = SimpleNamespace(priority=shared.get('priority', 2))
            team.append(SimpleNamespace(priority=shared.get('priority', 2),
                                        distance=shared.get('distance', np.inf),
                                        contest=shared.get('contest', False)))
        counters = (impc.prob.n_fail, impc.n_corridor_fail, impc.n_recover, len(impc.prob.solve_times))
        started = time.perf_counter()
        cpu_started = time.process_time()
        ok, path = self.paths.plan(state.position, goal)
        path_ms = (time.perf_counter() - started) * 1000
        if ok:
            impc.plan(goal, np.asarray(path, float)[:, :2], obstacles, others, team)
        nominal = ok and robot.mpc_fail_streak == 0
        solved = len(impc.prob.solve_times) > counters[3]
        metrics = dict(solver_ms=(time.perf_counter() - started) * 1000 - path_ms,
                       solver_cpu_ms=(time.process_time() - cpu_started) * 1000, path_ms=path_ms,
                       obstacle_points=len(obstacles), controller_mode=guidance['mode'],
                       impc_recovered=nominal and impc.n_recover > counters[2],
                       impc_fail_total=impc.prob.n_fail,
                       impc_corridor_fail_total=impc.n_corridor_fail,
                       impc_recover_total=impc.n_recover,
                       impc_priority=int(impc.priority), impc_eta=float(impc.eta),
                       impc_contest=bool(impc.contest),
                       impc_term_overlap=bool(impc.term_overlap))
        if solved:
            try:
                stats = impc.prob.opti.stats()
                metrics["solver_status"] = stats.get("return_status", "unknown")
                metrics["solver_iterations"] = stats.get("iter_count", 0)
            except RuntimeError:
                metrics["solver_status"] = "unavailable"
        else:
            metrics["solver_status"] = "not_solved"
        broadcast = dict(priority=int(impc.priority), distance=float(impc.distance),
                         contest=bool(impc.contest))
        if nominal:
            X = np.vstack([x0, impc._X[:-1]])
            U = np.array(impc.U, float)
            if not (np.isfinite(X).all() and np.isfinite(U).all()):
                return {"ok": False, "nominal_ok": False, "reason": "impc_invalid_output",
                        "broadcast": broadcast, **metrics}
            self.stop_plan = StopPlan(X.copy(), U.copy(), stamp)  # ends at rest
            self.own_plan = (X.copy(), stamp)
            return {"ok": True, "nominal_ok": True, "prediction": X, "controls": U,
                    "reason": "tracking" if guidance["mode"] == "TRACK" else "search",
                    "broadcast": broadcast, **metrics}
        failure = "impc_failed" if ok else "impc_planner_failed"
        tail = stop_tail(self.stop_plan, state, stamp, cfg)
        if tail is not None and tail[1] < H:
            plan, index = tail
            # upstream _tail_collides reads the remaining plan from the robot
            robot.states_prediction = to_upstream(plan.states[index:], z)
            if impc._tail_collides(obstacles):
                tail = None
        if tail is not None and tail[1] < H:
            X, U = brake_extend(plan.states[index:], plan.controls[index:], cfg)
            reason, retained = f"{failure}: previous_plan", True
        else:
            self.stop_plan = None
            X = impc._brake_traj(x0)
            U = np.array([impc._brake_u(row[2:4]) for row in X[:-1]])
            reason, retained = f"{failure}: brake", False
        if not (np.isfinite(X).all() and np.isfinite(U).all()):
            return {"ok": False, "nominal_ok": False, "reason": "impc_invalid_output",
                    "broadcast": broadcast, **metrics}
        self.own_plan = (X.copy(), stamp)
        robot.states_prediction = to_upstream(X, z)
        return {"ok": True, "nominal_ok": False, "prediction": X, "controls": U,
                "reason": reason, "retained_plan": retained, "broadcast": broadcast, **metrics}

    def payload_metrics(self):
        return {"impc_faces": self.impc.n_faces, "impc_warmup_ok": self.warmup_ok}

    def close(self):
        pass
