"""Baseline ``apf``: upstream apf.py artificial potential field per UAV.

Upstream APF runs unchanged on the shared leader/slot layers: the leader keeps
BASELINE_LEADER_STANDOFF from the predicted target, satellites fly to their
formation slots. Obstacles are the hits of the latest integrated LiDAR scan
(teammate returns already removed); teammates enter as upstream robots. No
path planner, corridor or retained stopping plan.
"""

import time

import numpy as np

from .. import upstream
from ..fleet import FleetAlgorithm
from .rollout import command_trajectory
from .shim import UpstreamRobot, from_upstream, neighbor_robots


class APFVehicle:
    def __init__(self, name, names, cfg, grid):
        self.name, self.names, self.cfg = name, list(names), dict(cfg)
        self.robot = UpstreamRobot(self.names.index(name))
        # One persistent upstream planner keeps upstream's per-robot RNG stream.
        self.apf = upstream.load('apf').APF(self.robot, self.cfg)

    def solve(self, states, predictions, guidance, stamp, scan):
        state = states[self.name]
        goal = guidance['goal'] if guidance['slot'] is None else guidance['slot']
        obstacles = (np.empty((0, 2)) if scan is None
                     else np.asarray(scan.endpoints, float)[np.asarray(scan.hits, bool), :2])
        robot = self.robot.measure(state)
        others = neighbor_robots(self.names, self.name, states, predictions)
        started = time.perf_counter()
        cpu_started = time.process_time()
        u = self.apf.plan(np.asarray(goal, float)[:2], obstacles, others)
        metrics = dict(solver_ms=(time.perf_counter() - started) * 1000,
                       solver_cpu_ms=(time.process_time() - cpu_started) * 1000,
                       obstacle_points=len(obstacles), controller_mode=guidance['mode'])
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        X, U = command_trajectory(x0, u[:2], self.cfg)
        prediction = from_upstream(robot.states_prediction)
        if not all(np.isfinite(value).all() for value in (u, X, U, prediction)):
            return {"ok": False, "nominal_ok": False, "reason": "apf_invalid_output", **metrics}
        return {"ok": True, "nominal_ok": True, "prediction": prediction,
                "trajectory": X, "controls": U,
                "reason": "tracking" if guidance["mode"] == "TRACK" else "search", **metrics}

    def payload_metrics(self):
        return {}

    def close(self):
        pass


class APFAlgorithm(FleetAlgorithm):
    NAME = 'apf'
    PLANNER = 'solo_mpc.core.baselines.apf:APFVehicle'
    GOAL_POLICY = 'baseline_standoff'
    UPSTREAM_MODULES = ('dynamics', 'apfc', 'apf')
