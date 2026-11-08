"""Geometry and scan validity shared by ROS adapters and offline tests."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


# Ideal gimbal: centered below the vehicle in world axes, with a fixed heading.
CAMERA_WORLD_OFFSET = (0.0, 0.0, -0.04)
# Optical right = world -Y, down = world -X, forward = world -Z.
CAMERA_OPTICAL_QUATERNION = tuple(Rotation.from_matrix(
    [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]).as_quat())


def stabilized_camera_pose(position):
    """World optical pose of the ideal gimbal, independent of body attitude."""
    return np.asarray(position, float) + CAMERA_WORLD_OFFSET, CAMERA_OPTICAL_QUATERNION


@dataclass
class ScanRays:
    origin: np.ndarray
    endpoints: np.ndarray
    hits: np.ndarray
    stamp: float


def project_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    position,
    quaternion,
    stamp,
    neighbors=(),
    neighbor_radius=0.5,
    ground_z=0.0,
    ground_tolerance=1e-3,
    ceiling_z=np.inf,
):
    """Only observed rays carve free space. NaN/zero/too-short rays remain unknown.

    Positive infinity and range_max are observed no-return rays. Rays ending on
    teammates retain their free prefix but never mark the teammate occupied.
    Reject an entire ray reaching the world ground plane (within 1 mm), including
    its free prefix, and likewise one ending at or above ceiling_z (the obstacle
    top), which may have passed over a building. A straight ray starting between
    them stays between them when its endpoint does. Remaining rays retain the full
    measured body attitude before the mapper projects them into XY.
    """
    ranges = np.asarray(ranges, float)
    valid = np.isposinf(ranges) | (
        np.isfinite(ranges) & (ranges > 0) & (ranges >= range_min) & (ranges <= range_max)
    )
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    lengths = np.minimum(ranges[valid], range_max)
    local = np.column_stack(
        [
            lengths * np.cos(angles[valid]),
            lengths * np.sin(angles[valid]),
            np.zeros(valid.sum()),
        ]
    )
    origin = np.asarray(position, float)
    endpoints = Rotation.from_quat(quaternion).apply(local) + origin
    hits = np.isfinite(ranges[valid]) & (ranges[valid] < range_max - 1e-3)
    in_band = (endpoints[:, 2] > ground_z + ground_tolerance) & (endpoints[:, 2] < ceiling_z)
    endpoints, hits = endpoints[in_band], hits[in_band]
    for p in neighbors:
        hits &= np.linalg.norm(endpoints - np.asarray(p), axis=1) > neighbor_radius
    return ScanRays(origin, endpoints, hits, stamp)


class PoseHistory:
    def __init__(self, tolerance=0.3, capacity=100):
        from collections import deque

        self.samples = deque(maxlen=capacity)
        self.tolerance = tolerance

    def add(self, stamp, position, quaternion):
        self.samples.append((stamp, np.asarray(position), np.asarray(quaternion)))

    def at(self, stamp):
        if not self.samples:
            raise ValueError("No measured pose available")
        sample = min(self.samples, key=lambda x: abs(x[0] - stamp))
        if abs(sample[0] - stamp) > self.tolerance:
            raise ValueError("No measured pose close to sensor exposure")
        return sample[1], sample[2]
