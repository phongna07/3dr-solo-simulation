"""Committed local grid paths for one vehicle, over its worker-owned map.

Refresh, trimming, deviation and previous-path fallback follow the upstream
robot_jps.py commit rules, independent of how a planner follows the path.
"""

import numpy as np

from .planner_grid import LocalGridPlanner, warmup


class PathService:
    def __init__(self, cfg, grid):
        self.cfg = cfg
        self.planner = LocalGridPlanner(
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
        self.planner.gmap = grid
        self.path, self.goal, self.age = None, None, 0
        self.failures = 0
        warmup()

    def plan(self, position, goal):
        planner = self.planner
        previous = self.path
        refresh = (
            previous is None or len(previous) < 2
            or self.age >= self.cfg["MAX_COMMIT_AGE"]
        )
        refresh |= (
            self.goal is None
            or np.linalg.norm(goal - self.goal)
            > self.cfg["TARGET_REPLAN_THRESHOLD"]
        )
        if not refresh:
            points = np.asarray(previous, float)
            nearest = self.segment(points, position[:2])
            if nearest + 2 < len(points) and np.linalg.norm(points[nearest + 1] - position[:2]) < 1e-6:
                nearest += 1
            trimmed = np.vstack(
                [position[:2], points[min(nearest + 1, len(points) - 1) :]]
            )
            delta = points[nearest + 1] - points[nearest]
            along = np.clip(np.dot(position[:2] - points[nearest], delta)
                            / max(np.dot(delta, delta), 1e-12), 0, 1)
            deviation = np.linalg.norm(position[:2] - points[nearest] - along * delta)
            refresh = deviation > self.cfg['PATH_DEVIATION_THRESHOLD'] or not all(
                planner.gmap.segment_clear(a, b)
                for a, b in zip(trimmed[:-1], trimmed[1:])
            )
            if not refresh:
                self.age += 1
                return True, trimmed.tolist()
        planner.initialize(position, goal)
        ok, path, *_ = planner.plan(
            np.empty((0, 2)), self.cfg["ROBOT_RADIUS"], observe=False
        )
        if ok and len(path) >= 2:
            self.failures = 0
            self.goal = np.asarray(goal).copy()
            self.age = 0
            self.path = path
            return True, path
        self.failures += 1
        if previous is not None and len(previous) >= 2:
            points = np.asarray(previous, float)
            k = self.segment(points, position[:2])
            trimmed = np.vstack([position[:2], points[k + 1:]])
            if all(planner.gmap.segment_clear(a, b) for a, b in zip(trimmed[:-1], trimmed[1:])):
                self.age += 1
                return True, trimmed.tolist()
        if self.failures < self.cfg['MAX_FAIL_BEFORE_STOP']:
            return True, [position[:2].copy(), position[:2].copy()]
        return False, []

    @staticmethod
    def segment(points, position):
        """Project onto segments so trimming a committed path cannot skip a turn."""
        delta = points[1:] - points[:-1]
        along = np.clip(np.sum((position - points[:-1]) * delta, axis=1)
                        / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0, 1)
        return int(np.argmin(np.linalg.norm(points[:-1] + along[:, None] * delta
                                           - position, axis=1)))
