"""Select the upstream CBF-QP baseline through the shared fleet runtime."""

from ...runtime.fleet import FleetAlgorithm


class CBFQPAlgorithm(FleetAlgorithm):
    NAME = 'cbf_qp'
    PLANNER = f'{__package__}.vehicle:CBFQPVehicle'
    GOAL_POLICY = 'baseline_standoff'
    UPSTREAM_MODULES = ('dynamics', 'apfc', 'cbfqp')
