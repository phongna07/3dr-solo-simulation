"""Timestamped target filter: prediction never masquerades as a detection."""

import numpy as np


class TargetTracker:
    def __init__(self):
        self.x = None
        self.P = np.eye(4)
        self.stamp = None
        self.observed_at = None

    def _predict(self, stamp):
        dt = max(0.0, stamp - self.stamp)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        B = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + B @ B.T * 4.0
        self.stamp = stamp

    def observe(self, position, stamp, variance=0.03):
        z = np.asarray(position)[:2]
        if self.x is None:
            self.x = np.r_[z, 0.0, 0.0]
            self.P = np.diag([variance, variance, 1.0, 1.0])
            self.stamp = stamp
        elif stamp > self.stamp:
            self._predict(stamp)
            H = np.eye(2, 4)
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + np.eye(2) * variance)
            self.x += K @ (z - H @ self.x)
            self.P = (np.eye(4) - K @ H) @ self.P
        else:
            return
        self.observed_at = stamp

    def trajectory(self, stamp, horizon, dt, max_speed):
        if self.x is None:
            raise ValueError("Target has never been observed")
        age = max(0.0, stamp - self.stamp)
        velocity = self.x[2:].copy()
        speed = np.linalg.norm(velocity)
        if speed > max_speed:
            velocity *= max_speed / speed
        # Do not extrapolate a vanished target indefinitely. SEARCH approaches
        # the last plausible location; only a real observation renews validity.
        coast = min(age, 1.5)
        velocity_scale = max(0.0, 1.0 - age / 1.5)
        return (
            self.x[:2]
            + coast * velocity
            + np.arange(horizon + 1)[:, None] * dt * velocity * velocity_scale
        )
