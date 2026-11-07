"""Select the upstream APF baseline through the shared fleet runtime."""

from ...runtime.fleet import FleetAlgorithm


class APFAlgorithm(FleetAlgorithm):
    NAME = 'apf'
    PLANNER = f'{__package__}.vehicle:APFVehicle'
    GOAL_POLICY = 'baseline_standoff'
    UPSTREAM_MODULES = ('dynamics', 'apfc', 'apf')
