"""Scenario configuration without import-time side effects or global state."""

import math
from pathlib import Path
import re
import warnings

import numpy as np
import yaml

from .core.target_motion import target_route

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
    LOCAL_SEARCH="jps",
    UNKNOWN_POLICY="optimistic",
    UNKNOWN_COST=1.5,
    GOAL_CLAMP_MARGIN=0.6,
    START_SNAP_RADIUS=10.0,
    TARGET_REPLAN_THRESHOLD=0.6,
    MAX_COMMIT_AGE=20,
    VIEWING_RADIUS=1.5,
    W_u=0.4,
    W_gui=1.0,
    W_tra=1.0,
    STANDOFF_DISTANCE=0.9,
    CORRIDOR_BARRIER_EPS=0.1,
    W_corridor=1.0,
    CORRIDOR_BOX=1.5,
    CORRIDOR_MAX_FACES=16,
    TERMINAL_STOP=False,
    CONTINGENCY=True,
    BACKUP_HORIZON=25,
    W_backup=0.1,
    BACKUP_POSITION_TOLERANCE=0.15,
    BACKUP_VELOCITY_TOLERANCE=0.3,
    BACKUP_ACCEL_TOLERANCE=0.5,
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
        "ipopt.max_iter": 500,
        "ipopt.max_cpu_time": 0.2,
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
    target_route(scenario, cfg)  # Reject malformed routes before any process starts.
    cfg["TARGET_MAX_SPEED"] = max(
        [float(cfg["VMAX"]), float(scenario["tar_max_speed"]),
         *scenario["target"].get("speeds", [])]
    )
    cfg["STANDOFF_DISTANCE"] = max(
        0.2, cfg["VIEWING_RADIUS"] - scenario["tar_max_speed"]
    )
    samples = settings["lidar_samples"]
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 4:
        raise ValueError("lidar_samples must be an integer of at least four")
    cfg["LIDAR_ANGULAR_RES"] = 2 * math.pi / samples
    noise = settings.get("lidar_noise_stddev", 0.01)
    if not math.isfinite(noise) or noise < 0:
        raise ValueError("lidar_noise_stddev must be finite and nonnegative")
    cfg.setdefault("LIDAR_NOISE", 3 * noise)
    # Upstream's sampling allowance plus cell-centre quantization: this port
    # decomposes occupied grid cells rather than the raw LiDAR endpoints.
    reach = cfg["VMAX"] * max(
        cfg["HORIZON_LENGTH"], cfg["BACKUP_HORIZON"] if cfg["CONTINGENCY"] else 0
    ) * cfg["TIMESTEP"]
    cfg.setdefault("CORRIDOR_MARGIN", cfg["ROBOT_RADIUS"] + cfg["LIDAR_NOISE"]
                   + reach * cfg["LIDAR_ANGULAR_RES"] / 2
                   + cfg["GRID_RESOLUTION"] / math.sqrt(2))
    cfg["CORRIDOR_SEED_MAX"] = cfg["SENSING_RADIUS"] - math.sqrt(2) * cfg["CORRIDOR_BOX"]
    validate(scenario, settings, cfg)
    return scenario, settings, cfg


def validate(scenario, settings, cfg):
    starts = np.asarray(scenario["starts"], float)
    if (
        starts.ndim != 2
        or len(starts) == 0
        or starts.shape[1] != 3
        or not np.isfinite(starts).all()
    ):
        raise ValueError("starts must be finite XYZ coordinates")
    if not np.allclose(starts[:, 2], settings["flight_altitude"], rtol=0, atol=1e-6):
        raise ValueError("starts altitude must match flight_altitude; select matching controller settings")
    for key in [
        "TIMESTEP",
        "VMAX",
        "UMAX",
        "ROBOT_RADIUS",
        "GRID_RESOLUTION",
        "SENSING_RADIUS",
        "CORRIDOR_BOX",
        "CORRIDOR_MARGIN",
        "START_SNAP_RADIUS",
        "BACKUP_POSITION_TOLERANCE",
        "BACKUP_VELOCITY_TOLERANCE",
        "BACKUP_ACCEL_TOLERANCE",
    ]:
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    for key in ("HORIZON_LENGTH", "BACKUP_HORIZON", "CORRIDOR_MAX_FACES"):
        if isinstance(cfg[key], bool) or not isinstance(cfg[key], int) or cfg[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    for key in ("TERMINAL_STOP", "CONTINGENCY"):
        if not isinstance(cfg[key], bool):
            raise ValueError(f"{key} must be a boolean")
    for key in ("W_backup", "LIDAR_NOISE"):
        if not math.isfinite(cfg[key]) or cfg[key] < 0:
            raise ValueError(f"{key} must be finite and nonnegative")
    if cfg["CORRIDOR_MARGIN"] < cfg["ROBOT_RADIUS"]:
        raise ValueError("CORRIDOR_MARGIN must be at least ROBOT_RADIUS")
    if cfg["CORRIDOR_SEED_MAX"] < cfg["GRID_RESOLUTION"]:
        raise ValueError("CORRIDOR_BOX leaves no corridor seed within SENSING_RADIUS")
    if cfg["LOCAL_SEARCH"] not in ("jps", "dijkstra"):
        raise ValueError("LOCAL_SEARCH must be jps or dijkstra")
    for enabled, horizon in (("CONTINGENCY", "BACKUP_HORIZON"),
                             ("TERMINAL_STOP", "HORIZON_LENGTH")):
        if cfg[enabled] and cfg["UMAX"] * (
            cfg[horizon] * cfg["TIMESTEP"] - cfg["ACCEL_TAU"] - cfg["TIMESTEP"]
        ) < cfg["VMAX"]:
            warnings.warn(f"{enabled}: {horizon} may limit speed below VMAX when stopping",
                          UserWarning, stacklevel=2)
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
    if not re.fullmatch(r"[a-z][a-z0-9_]*", scenario.get("world_name", "mpc_small50")):
        raise ValueError("Invalid scenario world_name")
    for key, default in (("LIDAR_RANGE_MIN", 0.15), ("TAR_RADIUS", 0.15),
                         ("KF_OBS_NOISE_STD", math.sqrt(0.03))):
        value = cfg.get(key, default)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if cfg.get("LIDAR_RANGE_MIN", 0.15) >= cfg["SENSING_RADIUS"]:
        raise ValueError("LIDAR_RANGE_MIN must be less than SENSING_RADIUS")
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
