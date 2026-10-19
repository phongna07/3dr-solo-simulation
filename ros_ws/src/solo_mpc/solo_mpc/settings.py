"""Scenario configuration without import-time side effects or global state."""

import math
from pathlib import Path

import numpy as np
import yaml

DEFAULTS = dict(
    HORIZON_LENGTH=10,
    TIMESTEP=0.1,
    ROBOT_RADIUS=0.36,
    VMAX=1.2,
    UMAX=2.0,
    D_FRAC=0.0,
    ACCEL_TAU=0.2,
    SENSING_RADIUS=3.0,
    SENSING_NEIGHBOR=5.0,
    GRID_RESOLUTION=0.15,
    INFLATE_MARGIN=0.15,
    LOCAL_PLAN_RADIUS=10.0,
    UNKNOWN_POLICY="optimistic",
    UNKNOWN_COST=1.5,
    GOAL_CLAMP_MARGIN=0.6,
    TARGET_REPLAN_THRESHOLD=0.6,
    MAX_COMMIT_AGE=20,
    VIEWING_RADIUS=1.5,
    W_u=0.4,
    W_gui=1.0,
    W_tra=1.0,
    STANDOFF_DISTANCE=0.9,
    CORRIDOR_BARRIER_EPS=0.1,
    W_corridor=1.0,
    CORRIDOR_BOX=2.0,
    CORRIDOR_MAX_FACES=16,
    COLLISION_AVOID_DISTANCE=1.5,
    W_collision_avoid=5.0,
    UAV_SAFE_DISTANCE=1.0,
    W_uav_slack=1e4,
    W_bounds_slack=1e4,
    W_leader_slack=1e3,
    CORRIDOR_RECOVER_ACCEL_RATIO=0.5,
    CORRIDOR_RECOVER_HOLD=0.3,
    CBF_BOX_RATIO=0.55,
    DT_CBF_GAMMA=0.5,
    COST_LENGTH_SCALE=1.0,
    COST_ACCEL_SCALE=1.0,
    MPC_SOLVER="ipopt",
    W_search_track=1.0,
    W_sat_slot=1.0,
    SLOT_SPACING_RATIO=1.75,
    SLOT_MIN_RATIO=0.45,
    SLOT_EXPAND_SPEED=0.5,
    GRID_CELLS=[[0, 1], [0, -1]],
    SWITCH_MARGIN=0.75,
    IPOPT_OPTIONS={
        "ipopt.max_iter": 100,
        "ipopt.max_cpu_time": 0.06,
        "ipopt.print_level": 0,
        "ipopt.tol": 1e-4,
        "print_time": 0,
    },
)


def load_settings(scenario_path, settings_path):
    scenario = yaml.safe_load(Path(scenario_path).read_text())
    settings = yaml.safe_load(Path(settings_path).read_text())
    cfg = {**DEFAULTS, **scenario.get("params", {}), **settings.get("params", {})}
    cfg["TIMESTEP"] = float(settings["control_period"])
    cfg["VIEWING_RADIUS"] = float(scenario["viewing_radius"])
    cfg["WORLD_BOUNDS"] = tuple(
        map(
            float,
            [
                scenario["xlim"][0],
                scenario["ylim"][0],
                scenario["xlim"][1],
                scenario["ylim"][1],
            ],
        )
    )
    cfg["INFLATE_RADIUS"] = cfg["ROBOT_RADIUS"] + cfg["INFLATE_MARGIN"]
    cfg["STANDOFF_DISTANCE"] = max(
        0.2, cfg["VIEWING_RADIUS"] - scenario["tar_max_speed"]
    )
    validate(scenario, settings, cfg)
    return scenario, settings, cfg


def validate(scenario, settings, cfg):
    for key in [
        "TIMESTEP",
        "VMAX",
        "UMAX",
        "ROBOT_RADIUS",
        "GRID_RESOLUTION",
        "SENSING_RADIUS",
    ]:
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if cfg["MPC_SOLVER"] != "ipopt":
        raise ValueError("This ROS port currently supports the validated IPOPT backend")
    if settings["target_mode"] not in ("camera", "ground_truth"):
        raise ValueError("target_mode must be camera or ground_truth")
    if settings.get("controller_execution", "parallel") not in ("parallel", "sequential"):
        raise ValueError("controller_execution must be parallel or sequential")
    for key, default in [("controller_worker_timeout_wall", 1.0),
                         ("controller_worker_startup_timeout_wall", 30.0)]:
        value = settings.get(key, default)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if (
        not settings["obstacle_height"]
        > settings["flight_altitude"] + cfg["ROBOT_RADIUS"]
    ):
        raise ValueError(
            "Fixed-altitude obstacles must exceed the vehicle flight envelope"
        )
    if not 0 < settings["camera_hfov"] < math.pi:
        raise ValueError("camera_hfov must be in radians between zero and pi")
    starts = np.asarray(scenario["starts"], float)
    if (
        starts.ndim != 2
        or len(starts) == 0
        or starts.shape[1] != 3
        or not np.isfinite(starts).all()
    ):
        raise ValueError("starts must be finite XYZ coordinates")
    for i, a in enumerate(starts):
        if any(
            np.linalg.norm(a[:2] - b[:2]) < cfg["UAV_SAFE_DISTANCE"]
            for b in starts[i + 1 :]
        ):
            raise ValueError("Initial UAV positions violate UAV_SAFE_DISTANCE")
    cells = np.asarray(cfg["GRID_CELLS"], float)
    if cells.size == 0:
        cells = cells.reshape(0, 2)
    if cells.ndim != 2 or cells.shape[1] != 2 or not np.isfinite(cells).all():
        raise ValueError("GRID_CELLS must be finite XY formation offsets")
    if len(cells) < len(starts) - 1:
        raise ValueError("GRID_CELLS needs at least one formation slot per follower UAV")
    if len({tuple(cell) for cell in cells}) != len(cells) or any(
        np.linalg.norm(cell) == 0 for cell in cells
    ):
        raise ValueError("GRID_CELLS offsets must be unique and nonzero")
