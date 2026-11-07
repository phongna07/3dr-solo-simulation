"""Baseline ``cbf_qp_lag``: upstream cbfqp.py with ``lag_aware=True``.

Every barrier also holds on the state T_d = ACCEL_TAU + TIMESTEP ahead.
"""

from ..cbf_qp.vehicle import CBFQPVehicle


class CBFQPLagVehicle(CBFQPVehicle):
    LAG_AWARE = True
