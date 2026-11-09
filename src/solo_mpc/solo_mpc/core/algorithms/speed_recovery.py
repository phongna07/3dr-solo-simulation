"""Measured-overspeed recovery shared by every algorithm with hard speed bounds.

Upstream writes |v_k| <= VMAX as constant rows, which a measured state above VMAX
makes infeasible. These helpers replace only those upper bounds with a parameter.
"""

import casadi as ca
import numpy as np

from .. import upstream as _sources

dynamics = _sources.load('dynamics')


def speed_recovery_limits(x0, cfg, steps):
    """Allow only initial overspeed a maximum-effort brake cannot remove."""
    state = np.asarray(x0, float).reshape(6).copy()
    A, B = dynamics.model(cfg['TIMESTEP'], cfg['D_FRAC'], cfg['ACCEL_TAU'], 2)
    direction = (A @ state)[2:4]
    control = -cfg['UMAX'] * direction / max(np.linalg.norm(direction), 1e-12)
    limits = np.full(steps, float(cfg['VMAX']))
    for k in range(steps):
        state = A @ state + B @ control
        speed = np.linalg.norm(state[2:4])
        if speed <= cfg['VMAX']:
            break
        limits[k] = speed + 1e-6
    return limits


def replace_speed_bounds(opti, params, branches, lower, upper, cfg):
    """Point the VMAX rows of each ``(state, count)`` branch at ``params['speed_limit']``."""
    params['speed_limit'] = opti.parameter(max(count for _, count in branches), 1)
    replaced = []
    for state, count in branches:
        if not count:
            continue
        rows = []
        for row, uses_state in enumerate(ca.which_depends(opti.g, state, 1, True)):
            if not uses_state:
                continue
            try:
                lo, hi = float(ca.evalf(lower[row])), float(ca.evalf(upper[row]))
            except RuntimeError:
                continue  # parameter-dependent bounds are not speed limits
            if lo == -np.inf and hi == cfg['VMAX'] ** 2:
                rows.append(row)
        if len(rows) != count:
            raise ValueError('Upstream speed constraints changed; review the solver adapter')
        replaced.extend((row, step) for step, row in enumerate(rows))
    # Assign last: a symbolic entry makes the bound vector non-numeric for evalf.
    for row, step in replaced:
        upper[row] = params['speed_limit'][step] ** 2


def enable_speed_recovery(opti, params, state, count, cfg):
    """Install recovery on an already built upstream baseline problem, if configured."""
    if not cfg.get('SPEED_RECOVERY', False):
        return
    lower, upper = opti.lbg, opti.ubg
    constraints = opti.g
    replace_speed_bounds(opti, params, ((state, count),), lower, upper, cfg)
    opti.subject_to()  # Keep every upstream constraint, replacing only the speed bounds.
    opti.subject_to(opti.bounded(lower, constraints, upper))


def set_speed_limit(opti, params, x0, cfg, steps):
    if 'speed_limit' in params:
        opti.set_value(params['speed_limit'], speed_recovery_limits(x0, cfg, steps))
