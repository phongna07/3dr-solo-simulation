"""Select the upstream IMPC-OB (recover) baseline through the shared fleet runtime."""

from ...runtime.contracts import VehicleResult, VehicleState
from ...runtime.fleet import FleetAlgorithm
from ..stop_plan import revalidate_retained


class IMPCOBRecAlgorithm(FleetAlgorithm):
    NAME = 'impc_ob_rec'
    PLANNER = f'{__package__}.vehicle:IMPCOBRecVehicle'
    GOAL_POLICY = 'baseline_standoff'
    # IMPC-OB plans end at rest; upstream looks ahead along the leader's velocity.
    SLOT_LOOKAHEAD = 'current_velocity'
    UPSTREAM_MODULES = ('dynamics', 'impc_ob')

    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult:
        """A replayed previous plan must still match the collection-time measurement."""
        return revalidate_retained(result, state, self.controller.cfg)
