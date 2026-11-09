"""Executable rows for a baseline's one-step acceleration command.

Upstream applies a baseline's command for one TIMESTEP, then plans again. Step 0
applies that command with upstream's predictive dynamics. Delivery latency
shifts setpoint sampling up to one TIMESTEP later, so a baseline command is held
for a second step (unless that would exceed VMAX) to keep sampling off the
braking tail. Later steps brake to rest with upstream impc_ob._brake_u, which a
missed cycle flies until setpoint expiry.
"""

import numpy as np

from .. import upstream


def brake(velocity, cfg):
    """Upstream impc_ob._brake_u: cancel velocity over ACCEL_TAU + TIMESTEP, at most UMAX."""
    u = -np.asarray(velocity, float) / (cfg["ACCEL_TAU"] + cfg["TIMESTEP"])
    n = np.linalg.norm(u)
    return u * (cfg["UMAX"] / n) if n > cfg["UMAX"] else u


def command_trajectory(x0, u, cfg, hold_steps=1):
    """States (H+1, 6) [px, py, vx, vy, ax, ay] and controls (H, 2) from x0."""
    A, B = upstream.load('dynamics').model(cfg["TIMESTEP"], cfg["D_FRAC"], cfg["ACCEL_TAU"], 2)
    X, U = [np.asarray(x0, float)], []
    for step in range(cfg["HORIZON_LENGTH"]):
        control = np.asarray(u, float)[:2]
        if step > 0 and (step >= hold_steps
                         or np.linalg.norm((A @ X[-1] + B @ control)[2:4]) > cfg["VMAX"]):
            control = brake(X[-1][2:4], cfg)
        U.append(control)
        X.append(A @ X[-1] + B @ control)
    return np.array(X), np.array(U)


def brake_extend(states, controls, cfg):
    """Extend a plan tail to H+1 rows and H controls by braking from its last row."""
    A, B = upstream.load('dynamics').model(cfg["TIMESTEP"], cfg["D_FRAC"], cfg["ACCEL_TAU"], 2)
    H = cfg["HORIZON_LENGTH"]
    X = [np.asarray(row, float) for row in states[:H + 1]]
    U = [np.asarray(row, float) for row in controls[:len(X) - 1]]
    while len(X) < H + 1:
        U.append(brake(X[-1][2:4], cfg))
        X.append(A @ X[-1] + B @ U[-1])
    return np.array(X), np.array(U)
