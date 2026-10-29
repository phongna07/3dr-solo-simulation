"""ROS-side compatibility adapter over the unchanged upstream MPCProblem.

The objective, dynamics, contingency branch, solver selection and solve routine
come directly from the submodule. Only the existing ROS-specific conditioning
of inactive neighbors and optional measured overspeed recovery are applied here.
"""

import casadi as ca
import numpy as np

from .upstream import dynamics, solver as upstream

BIG = upstream.BIG


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


class MPCProblem(upstream.MPCProblem):
    def __init__(self, n_others, n_faces, cfg):
        # Construct fallback through this adapter too, without altering upstream's
        # module globals or building an unused upstream fallback NLP.
        super().__init__(n_others, n_faces, {**cfg, 'SQP_FALLBACK_IPOPT': False})
        self.cfg = dict(cfg)
        o = self.opti
        constraints = ca.vertsplit(o.g)
        lower, upper = o.lbg, o.ubg
        neighbor_rows = [i for i, depends in enumerate(
            ca.which_depends(o.g, self.p['nb_flag'], 1, True)) if depends]
        H = cfg['HORIZON_LENGTH']
        if len(neighbor_rows) != H * n_others:
            raise ValueError('Upstream neighbor constraints changed; review the solver adapter')
        for index, row in enumerate(neighbor_rows):
            step, neighbor = divmod(index, n_others)
            flag = self.p['nb_flag'][neighbor]
            # Upstream normalizes these as g <= 0. For binary flags the active
            # constraint is unchanged; the inactive one is -Sc - Dsafe² <= 0.
            # This avoids a large inactive distance in IPOPT's barrier system.
            if float(ca.evalf(upper[row])) != 0 or float(ca.evalf(lower[row])) != -np.inf:
                raise ValueError('Upstream neighbor bound convention changed; review the adapter')
            constraints[row] = (flag * constraints[row] - (1 - flag) *
                                (self.Sc[step, neighbor] + cfg['UAV_SAFE_DISTANCE'] ** 2))
        if cfg.get('SPEED_RECOVERY', False):
            self.p['speed_limit'] = o.parameter(max(H, self.Hb), 1)
            speed_bounds = []
            for state, count in ((self.X, H), (self.Xb, self.Hb)):
                if not count:
                    continue
                depends = ca.which_depends(o.g, state, 1, True)
                rows = []
                for row, uses_state in enumerate(depends):
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
                for step, row in enumerate(rows):
                    speed_bounds.append((row, step))
            for row, step in speed_bounds:
                upper[row] = self.p['speed_limit'][step] ** 2
        if neighbor_rows or 'speed_limit' in self.p:
            o.subject_to()  # Keep every upstream constraint, replacing only the above bounds/rows.
            o.subject_to(o.bounded(lower, ca.vertcat(*constraints), upper))
        if cfg.get('MPC_SOLVER', 'ipopt') == 'sqp' and cfg.get('SQP_FALLBACK_IPOPT', True):
            self.fallback = type(self)(n_others, n_faces, {**cfg, 'MPC_SOLVER': 'ipopt'})

    def _solve(self, *args, **kwargs):
        flags = kwargs['nb_flags'] if 'nb_flags' in kwargs else args[7]
        if not np.isin(flags, (0.0, 1.0)).all():
            raise ValueError('Neighbor activation flags must be binary')
        if 'speed_limit' in self.p:
            x0 = kwargs['x0'] if 'x0' in kwargs else args[0]
            self.opti.set_value(self.p['speed_limit'], speed_recovery_limits(
                x0, self.cfg, max(self.cfg['HORIZON_LENGTH'], self.Hb)))
        return super()._solve(*args, **kwargs)
