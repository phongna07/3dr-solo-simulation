"""The upstream robot_jps.Robot attributes that baseline planners read.

Upstream baselines take a robot object and its teammates. Here each shim holds
only the frozen measured snapshot of one fleet cycle, never simulated state.
"""

import numpy as np


class UpstreamRobot:
    """``index`` is the fleet order (upstream's 0-based robot index and RNG seed)."""

    def __init__(self, index):
        self.index = index
        self.state = np.zeros(6)        # [x, y, z, vx, vy, vz]
        self.accel = np.zeros(3)
        self.states_prediction = None   # rows [x, y, z, vx, vy, 0]
        self.mpc_fail_streak = 0

    def measure(self, state):
        """Measured world ENU VehicleState; acceleration is the filtered estimate."""
        self.state = np.r_[np.asarray(state.position, float)[:3],
                           np.asarray(state.velocity, float)[:3]]
        self.accel = np.asarray(state.acceleration, float)[:3].copy()
        return self


def neighbor_robots(names, name, states, predictions):
    """Teammate shims from the frozen cycle snapshot, as upstream ``others``."""
    others = []
    for index, other in enumerate(names):
        if other == name:
            continue
        robot = UpstreamRobot(index).measure(states[other])
        robot.states_prediction = to_upstream(predictions[other], robot.state[2])
        others.append(robot)
    return others


def to_upstream(rows, z):
    """Local [px, py, vx, vy, ax, ay] rows -> upstream [x, y, z, vx, vy, 0]."""
    rows = np.asarray(rows, float)
    count = len(rows)
    return np.column_stack([rows[:, :2], np.full(count, z), rows[:, 2:4], np.zeros(count)])


def from_upstream(rows):
    """Upstream [x, y, z, vx, vy, vz] rows -> local rows; upstream carries no acceleration."""
    rows = np.asarray(rows, float)
    return np.column_stack([rows[:, :2], rows[:, 3:5], np.zeros((len(rows), 2))])
