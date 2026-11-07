"""Sample a target route with one positive speed per waypoint leg."""

import numpy as np

from .geometry import catmull_rom

# The mission completes, and pauses the target, within this XY goal distance.
GOAL_TOLERANCE = 0.05


def startup_route_time(elapsed, ramp_seconds):
    """Ramp the route clock from rest; pauses retain the same elapsed clock."""
    elapsed = max(0.0, elapsed)
    if ramp_seconds > 0 and elapsed < ramp_seconds:
        return elapsed * elapsed / (2 * ramp_seconds)
    return elapsed - ramp_seconds / 2


def target_route(scenario, cfg):
    target = scenario["target"]
    points = np.asarray(target["waypoints"], float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2
            or not np.isfinite(points).all()
            or np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) <= 0)):
        raise ValueError("target.waypoints must contain distinct consecutive finite XY points")
    base_speed = float(scenario["tar_max_speed"])
    if not np.isfinite(base_speed) or base_speed <= 0:
        raise ValueError("tar_max_speed must be finite and positive")
    speeds = np.asarray(target.get("speeds", [base_speed] * (len(points) - 1)), float)
    if (speeds.shape != (len(points) - 1,) or not np.isfinite(speeds).all()
            or np.any(speeds <= 0)):
        raise ValueError("target.speeds needs one finite positive speed per waypoint leg")
    ds = float(cfg.get("TAR_SPLINE_DS", 0.1))
    if not np.isfinite(ds) or ds <= 0:
        raise ValueError("TAR_SPLINE_DS must be finite and positive")
    if cfg.get("TAR_SMOOTH_ENABLE", True):
        path, knots = catmull_rom(points, ds)
    else:
        path, knots = points.copy(), np.arange(len(points))
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    durations = lengths / np.repeat(speeds, np.diff(knots))
    times = np.r_[0.0, np.cumsum(durations)]
    # Route-clock time at which the target reaches each scenario waypoint.
    return path, times, times[knots]
