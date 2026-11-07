"""Select the upstream plain MPC baseline through the shared fleet runtime."""

from ...runtime.contracts import VehicleResult, VehicleState
from ...runtime.fleet import FleetAlgorithm
from ..stop_plan import revalidate_retained


class PureMPCAlgorithm(FleetAlgorithm):
    NAME = 'pure_mpc'
    PLANNER = f'{__package__}.vehicle:PureMPCVehicle'
    GOAL_POLICY = 'baseline_standoff'
    UPSTREAM_MODULES = ('dynamics', 'pure_mpc')

    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult:
        """A replayed previous plan must still match the collection-time measurement."""
        return revalidate_retained(result, state, self.controller.cfg)
