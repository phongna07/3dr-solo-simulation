"""Baseline ``cbf_qp``: upstream cbfqp.py safety barrier certificates per UAV.

Upstream CBFQP filters a PD controller to the shared leader/slot goal, with the
target filter velocity as feed-forward. Obstacles are the hits of the latest
integrated LiDAR scan; teammates enter as upstream robots with measured
acceleration. An infeasible QP brakes (upstream hybrid braking) and is reported
as the visible fallback ``cbf_infeasible_brake`` from upstream's own counters.
"""

import time

import numpy as np

from ... import upstream
from ..command_rollout import command_trajectory
from ..upstream_robot import UpstreamRobot, from_upstream, neighbor_robots


class CBFQPVehicle:
    LAG_AWARE = False

    def __init__(self, name, names, cfg, grid):
        self.name, self.names, self.cfg = name, list(names), dict(cfg)
        self.robot = UpstreamRobot(self.names.index(name))
        # One persistent upstream filter per worker keeps its counters.
        self.cbf = upstream.load('cbfqp').CBFQP(self.robot, self.cfg, lag_aware=self.LAG_AWARE)

    def solve(self, states, predictions, guidance, stamp, scan):
        state = states[self.name]
        goal = guidance['goal'] if guidance['slot'] is None else guidance['slot']
        obstacles = (np.empty((0, 2)) if scan is None
                     else np.asarray(scan.endpoints, float)[np.asarray(scan.hits, bool), :2])
        robot = self.robot.measure(state)
        others = neighbor_robots(self.names, self.name, states, predictions)
        infeasible, deadlocks = self.cbf.n_infeasible, self.cbf.n_deadlock
        started = time.perf_counter()
        cpu_started = time.process_time()
        u = self.cbf.plan(np.asarray(goal, float)[:2],
                          np.asarray(guidance['target_velocity'], float)[:2], obstacles, others)
        braked = self.cbf.n_infeasible > infeasible
        metrics = dict(solver_ms=(time.perf_counter() - started) * 1000,
                       solver_cpu_ms=(time.process_time() - cpu_started) * 1000,
                       obstacle_points=len(obstacles), controller_mode=guidance['mode'],
                       cbf_deadlock=self.cbf.n_deadlock > deadlocks,
                       cbf_infeasible_total=self.cbf.n_infeasible,
                       cbf_deadlock_total=self.cbf.n_deadlock)
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        X, U = command_trajectory(x0, u[:2], self.cfg)
        prediction = from_upstream(robot.states_prediction)
        if not all(np.isfinite(value).all() for value in (u, X, U, prediction)):
            return {"ok": False, "nominal_ok": False, "reason": "cbf_invalid_output", **metrics}
        reason = ("cbf_infeasible_brake" if braked
                  else "tracking" if guidance["mode"] == "TRACK" else "search")
        return {"ok": True, "nominal_ok": not braked, "prediction": prediction,
                "trajectory": X, "controls": U, "reason": reason, **metrics}

    def payload_metrics(self):
        return {}

    def close(self):
        pass
