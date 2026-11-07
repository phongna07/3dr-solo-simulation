"""Translate existing MPC plans and diagnostics into runtime contracts."""

from dataclasses import replace

from ...runtime.contracts import VehicleResult, VehicleState
from ...runtime.fleet import FleetAlgorithm


class MPCAlgorithm(FleetAlgorithm):
    NAME = 'mpc'
    PLANNER = f'{__package__}.vehicle:MPCVehicle'
    GOAL_POLICY = 'pursuit'
    UPSTREAM_MODULES = ('dynamics', 'mpc_problem')

    def revalidate(self, result: VehicleResult, state: VehicleState) -> VehicleResult:
        """Preserve MPC's retained-stop measured-state check at collection."""
        if not result.publication_eligible or not result.fallback_used:
            return result
        import numpy as np
        prediction = result.prediction
        rows = prediction.states
        velocity_stamp = state.stamp if state.velocity_stamp is None else state.velocity_stamp
        indices = [min(int(max(0., stamp - prediction.source_stamp) /
                           prediction.timestep + 1e-6), len(rows)-1)
                   for stamp in (state.stamp, velocity_stamp, velocity_stamp)]
        measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
        expected = [rows[index, 2 * component:2 * component + 2]
                    for component, index in enumerate(indices)]
        matches = all(np.isfinite(value).all() and np.linalg.norm(value - target) <= self.controller.cfg[key]
                      for value, target, key in zip(measured, expected,
                          ('BACKUP_POSITION_TOLERANCE', 'BACKUP_VELOCITY_TOLERANCE', 'BACKUP_ACCEL_TOLERANCE')))
        return result if matches else replace(
            result, publication_eligible=False, reason='stop_plan_measured_state_changed')
