"""ROS-side compatibility adapter over the unchanged upstream MPCProblem.

The objective, dynamics, contingency branch, solver selection and solve routine
come directly from the submodule. Local conditioning and a bounded, per-vehicle
cache avoid paying for unused corridor rows without changing actual faces.
"""

from collections import OrderedDict

import casadi as ca
import numpy as np

from ... import upstream as _sources
from ..speed_recovery import replace_speed_bounds, set_speed_limit

UPSTREAM_MODULES = ('dynamics', 'mpc_problem')
upstream = _sources.load('mpc_problem')
BIG = upstream.BIG


class SolverCache:
    """Worker-owned exact face counts; no solver or warm start is fleet-shared.

    Keep anticipated sizes resident and reserve two LRU slots for unusual
    corridors. Cache misses retain every face and pay construction cost within
    the existing worker deadline, rather than silently simplifying geometry.
    """

    def __init__(self, n_others, cfg):
        self.n_others, self.cfg = n_others, dict(cfg)
        self.anticipated = frozenset((*cfg.get('CORRIDOR_CACHE_FACES', range(4, 9)),
                                     int(cfg['CORRIDOR_MAX_FACES'])))
        if min(self.anticipated) < 1:
            raise ValueError('Corridor solver capacities must be positive')
        self.limit = len(self.anticipated) + 2
        self.instances = OrderedDict()
        self.active = None
        # Opti defers expansion and native solver initialization until solve().
        # Do that before the worker announces readiness, including fallback.
        for faces in sorted(self.anticipated):
            solver = MPCProblem(n_others, faces, self.cfg)
            self._initialize(solver)
            self.instances[faces] = solver

    @staticmethod
    def _reset(solver):
        # Nominal X/U guesses belong to the controller and have size independent
        # of face count. Backup guesses belong to an instance and may be old.
        solver._bk_init = None
        solver.last_backup = None
        if solver.fallback is not None:
            SolverCache._reset(solver.fallback)

    def _initialize(self, solver):
        cfg = self.cfg
        H = cfg['HORIZON_LENGTH']
        bounds = np.asarray(cfg['WORLD_BOUNDS'], float)
        center = (bounds[:2] + bounds[2:]) / 2
        x0 = np.r_[center, np.zeros(4)]
        inputs = dict(
            x0=x0, A_hard=None, b_hard=None, A_cost=None, b_cost=None,
            use_corr_cost=False,
            others=[np.tile(center + (j + 1) * (2 * cfg['COLLISION_AVOID_DISTANCE'] + 1),
                            (H + 1, 1)) for j in range(self.n_others)],
            nb_flags=np.zeros(self.n_others), tgt=center, w_search=0.,
            slot=center, w_slot=0., leader=False, ref=center, trk_goal=center,
            s_ref=0., X_init=np.tile(x0, (H + 1, 1)), U_init=np.zeros((H, 2)), w_tra=0.,
        )
        current = solver
        while current is not None:
            # Use _solve so SQP success cannot leave its IPOPT fallback cold.
            ok, X, U = current._solve(**inputs)
            if not ok or not np.isfinite(X).all() or not np.isfinite(U).all():
                raise RuntimeError(f'Preflight initialization failed for {solver.n_faces} faces')
            current = current.fallback
        self._reset(solver)

    def select(self, hard_faces, cost_faces):
        faces = max(1, hard_faces, cost_faces)
        solver = self.instances.get(faces)
        if solver is None:
            if len(self.instances) >= self.limit:
                victim = next(size for size in self.instances if size not in self.anticipated)
                del self.instances[victim]
                # Drop the active reference too if its instance was evicted.
                if self.active is not None and self.active.n_faces == victim:
                    self.active = None
            solver = MPCProblem(self.n_others, faces, self.cfg)
            self.instances[faces] = solver
        self.instances.move_to_end(faces)
        if solver is not self.active:
            self._reset(solver)
        self.active = solver
        return solver


class MPCProblem(upstream.MPCProblem):
    UPSTREAM_MODULES = UPSTREAM_MODULES

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
            replace_speed_bounds(o, self.p, ((self.X, H), (self.Xb, self.Hb)), lower, upper, cfg)
        if neighbor_rows or 'speed_limit' in self.p:
            o.subject_to()  # Keep every upstream constraint, replacing only the above bounds/rows.
            o.subject_to(o.bounded(lower, ca.vertcat(*constraints), upper))
        if cfg.get('MPC_SOLVER', 'ipopt') == 'sqp' and cfg.get('SQP_FALLBACK_IPOPT', True):
            self.fallback = type(self)(n_others, n_faces, {**cfg, 'MPC_SOLVER': 'ipopt'})

    def _solve(self, *args, **kwargs):
        flags = kwargs['nb_flags'] if 'nb_flags' in kwargs else args[7]
        if not np.isin(flags, (0.0, 1.0)).all():
            raise ValueError('Neighbor activation flags must be binary')
        x0 = kwargs['x0'] if 'x0' in kwargs else args[0]
        set_speed_limit(self.opti, self.p, x0, self.cfg, max(self.cfg['HORIZON_LENGTH'], self.Hb))
        return super()._solve(*args, **kwargs)
