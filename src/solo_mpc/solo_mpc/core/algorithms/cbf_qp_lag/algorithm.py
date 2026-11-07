"""Select the upstream lag-aware CBF-QP baseline through the shared fleet runtime."""

from ...runtime.fleet import FleetAlgorithm


class CBFQPLagAlgorithm(FleetAlgorithm):
    NAME = 'cbf_qp_lag'
    PLANNER = f'{__package__}.vehicle:CBFQPLagVehicle'
    GOAL_POLICY = 'baseline_standoff'
    UPSTREAM_MODULES = ('dynamics', 'apfc', 'cbfqp')
