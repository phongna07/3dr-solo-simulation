"""Translate existing MPC plans and diagnostics into runtime contracts."""

from ...runtime.contracts import VehicleResult, VehicleState
from ...runtime.fleet import FleetAlgorithm
from ..stop_plan import revalidate_retained


class MPCAlgorithm(FleetAlgorithm):
    NAME = 'mpc'
    PLANNER = f'{__package__}.vehicle:MPCVehicle'
    GOAL_POLICY = 'pursuit'
    UPSTREAM_MODULES = ('dynamics', 'mpc_problem')

    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult:
        """Preserve MPC's retained-stop measured-state check at collection."""
        return revalidate_retained(result, state, self.controller.cfg)
