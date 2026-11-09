"""Baseline ``pure_mpc``: upstream pure_mpc.py plain MPC per UAV.

Upstream PureMPC (no path planner, corridor or CBF) runs unchanged on the shared
leader/slot layers and is built once per worker, before readiness. Obstacles
are the hits of the latest integrated LiDAR scan; teammates enter as upstream
robots in fleet order, which binds upstream's plan parameter j to teammate j.
The solved plan is executed directly. A failed solve ignores upstream's
counter-indexed replay command and flies the previous plan by simulation time
while the measured state agrees with it (extended by braking), else brakes; both
are visible fallbacks.
"""

import time

import numpy as np

from ... import upstream
from ..command_rollout import brake, brake_extend, command_trajectory
from ..speed_recovery import enable_speed_recovery, set_speed_limit
from ..stop_plan import StopPlan, stop_tail
from ..upstream_robot import UpstreamRobot, neighbor_robots


class PureMPCVehicle:
    def __init__(self, name, names, cfg, grid):
        self.name, self.names, self.cfg = name, list(names), dict(cfg)
        self.robot = UpstreamRobot(self.names.index(name))
        self.mpc = upstream.load('pure_mpc').PureMPC(self.robot, len(self.names) - 1, self.cfg)
        enable_speed_recovery(self.mpc.opti, self.mpc.p, self.mpc.X, self.cfg["HORIZON_LENGTH"], self.cfg)
        self.stop_plan = None

    def solve(self, states, predictions, guidance, stamp, scan):
        cfg = self.cfg
        H = cfg["HORIZON_LENGTH"]
        state = states[self.name]
        goal = guidance['goal'] if guidance['slot'] is None else guidance['slot']
        obstacles = (np.empty((0, 2)) if scan is None
                     else np.asarray(scan.endpoints, float)[np.asarray(scan.hits, bool), :2])
        self.robot.measure(state)
        set_speed_limit(self.mpc.opti, self.mpc.p,
                        np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]], cfg, H)
        others = neighbor_robots(self.names, self.name, states, predictions)
        fails = self.mpc.n_fail
        started = time.perf_counter()
        cpu_started = time.process_time()
        self.mpc.plan(np.asarray(goal, float)[:2], obstacles, others)
        metrics = dict(solver_ms=(time.perf_counter() - started) * 1000,
                       solver_cpu_ms=(time.process_time() - cpu_started) * 1000,
                       obstacle_points=len(obstacles), controller_mode=guidance['mode'],
                       pure_mpc_fail_total=self.mpc.n_fail)
        try:
            stats = self.mpc.opti.stats()
            metrics["solver_status"] = stats.get("return_status", "unknown")
            metrics["solver_iterations"] = stats.get("iter_count", 0)
        except RuntimeError:
            metrics["solver_status"] = "unavailable"
        solved = self.mpc.n_fail == fails
        if solved:
            X, U = np.array(self.mpc.Xp, float), np.array(self.mpc.Up, float)
            if not (np.isfinite(X).all() and np.isfinite(U).all()):
                return {"ok": False, "nominal_ok": False, "reason": "pure_mpc_invalid_output", **metrics}
            self.stop_plan = StopPlan(X.copy(), U.copy(), stamp)
            return {"ok": True, "nominal_ok": True, "prediction": X, "controls": U,
                    "reason": "tracking" if guidance["mode"] == "TRACK" else "search", **metrics}
        tail = stop_tail(self.stop_plan, state, stamp, cfg)
        if tail is not None and tail[1] < H:
            plan, index = tail
            X, U = brake_extend(plan.states[index:], plan.controls[index:], cfg)
            reason, retained = "pure_mpc_failed: previous_plan", True
        else:
            self.stop_plan = None
            x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
            X, U = command_trajectory(x0, brake(x0[2:4], cfg), cfg)
            reason, retained = "pure_mpc_failed: brake", False
        if not (np.isfinite(X).all() and np.isfinite(U).all()):
            return {"ok": False, "nominal_ok": False, "reason": "pure_mpc_invalid_output", **metrics}
        return {"ok": True, "nominal_ok": False, "prediction": X, "controls": U,
                "reason": reason, "retained_plan": retained, **metrics}

    def payload_metrics(self):
        return {}

    def close(self):
        pass
