"""ROS-free measured inputs and source-stamped planning/execution contracts.

CycleRunner deep-copies inputs before asynchronous execution. Frozen containers
express ownership; arrays are owned by that snapshot, never live ROS messages.
"""

from dataclasses import dataclass, field
import importlib
from typing import Protocol, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .sensors import ScanRays


@dataclass
class VehicleState:
    """Measured world ENU state; stamp is the original pose source epoch."""
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    quaternion: np.ndarray
    stamp: float
    velocity_stamp: float | None = None


class TargetEstimate(Protocol):
    observed_at: float | None
    x: np.ndarray | None

    def observe(self, position, stamp, variance=None) -> bool: ...
    def position(self, stamp) -> np.ndarray: ...
    def trajectory(self, stamp, horizon, dt, max_speed) -> np.ndarray: ...
    def pursuit(self, position, stamp, cfg) -> np.ndarray: ...


@dataclass(frozen=True)
class CycleInput:
    states: dict[str, VehicleState]
    scans: dict[str, 'ScanRays']
    source_stamp: float
    seers: set[str]
    tracker: TargetEstimate


@dataclass(frozen=True)
class ExecutableTrajectory:
    """World XY references; acceleration is actuator intent, not measured state."""
    source_stamp: float
    timestep: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


@dataclass(frozen=True)
class NeighborPrediction:
    """Planning-only world XY position/velocity/acceleration rows."""
    source_stamp: float
    timestep: float
    states: np.ndarray


@dataclass(frozen=True)
class VehicleResult:
    nominal_success: bool
    fallback_used: bool
    publication_eligible: bool
    reason: str
    trajectory: ExecutableTrajectory | None = None
    prediction: NeighborPrediction | None = None
    diagnostics: dict = field(default_factory=dict)


@dataclass(frozen=True)
class CycleResult:
    source_stamp: float
    vehicles: dict[str, VehicleResult]
    coordination: dict


class VehiclePlanner(Protocol):
    """One UAV's numerical planner, constructed inside its persistent worker.

    ``grid`` is the worker-owned SensorGrid; ``scan`` is the latest integrated
    ScanRays. ``solve`` returns the worker result dict (``ok``, ``reason``,
    ``prediction``, ``controls`` and optional ``trajectory``/``nominal_ok``).
    """

    def __init__(self, name, names, cfg, grid): ...
    def solve(self, states, predictions, guidance, stamp, scan) -> dict: ...
    def payload_metrics(self) -> dict: ...
    def close(self) -> None: ...


class Algorithm(Protocol):
    UPSTREAM_MODULES: tuple[str, ...]
    track: TargetEstimate

    def step(self, inputs: CycleInput) -> CycleResult: ...
    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult: ...
    def occupancy_maps(self) -> list: ...
    def diagnostics(self) -> dict: ...
    def close(self) -> None: ...


# Name -> (module, class), imported only when selected.
ALGORITHMS = {'mpc': ('.mpc_algorithm', 'MPCAlgorithm'),
              'apf': ('.baselines.apf', 'APFAlgorithm')}


def algorithm_names():
    return tuple(ALGORITHMS)


def algorithm_class(name):
    if name not in ALGORITHMS:
        raise ValueError(f'Unknown algorithm: {name}; use one of {", ".join(ALGORITHMS)}')
    module, attribute = ALGORITHMS[name]
    return getattr(importlib.import_module(module, __package__), attribute)


def create_algorithm(name, names, cfg, settings) -> Algorithm:
    return algorithm_class(name)(names, cfg, settings)
