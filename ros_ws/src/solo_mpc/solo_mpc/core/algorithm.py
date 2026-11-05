"""ROS-free boundary between planning algorithms and measured-state execution."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class VehicleState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    quaternion: np.ndarray
    stamp: float


class Algorithm(Protocol):
    track: object

    def step(self, states, scans, stamp, seers, *, tracker=None) -> dict: ...

    def occupancy_maps(self) -> list: ...

    def diagnostics(self) -> dict: ...

    def close(self) -> None: ...


def create_algorithm(name, names, cfg, settings) -> Algorithm:
    if name != 'mpc':
        raise ValueError(f'Unknown algorithm: {name}')
    from .mpc_algorithm import MPCAlgorithm

    return MPCAlgorithm(names, cfg, settings)
