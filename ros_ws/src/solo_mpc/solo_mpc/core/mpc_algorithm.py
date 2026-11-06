"""Translate existing MPC plans and diagnostics into runtime contracts."""

from dataclasses import replace

from .algorithm import (CycleInput, CycleResult, ExecutableTrajectory,
                        NeighborPrediction, VehicleResult, VehicleState)
from .parallel_controller import ParallelTeamController


class MPCAlgorithm:
    def __init__(self, names, cfg, settings):
        self.controller = ParallelTeamController(
            names, cfg,
            timeout=settings.get('controller_worker_timeout_wall', 1.0),
            startup_timeout=settings.get('controller_worker_startup_timeout_wall', 30.0),
            sequential=settings.get('controller_execution', 'parallel') == 'sequential',
        )

    @property
    def track(self):
        return self.controller.track

    @track.setter
    def track(self, tracker):
        self.controller.track = tracker

    def step(self, inputs: CycleInput) -> CycleResult:
        raw = self.controller.step(inputs.states, inputs.scans, inputs.source_stamp,
                                   inputs.seers, tracker=inputs.tracker)
        dt = self.controller.cfg['TIMESTEP']
        vehicles = {}
        for name, out in raw.items():
            rows, controls = out.get('prediction'), out.get('controls')
            prediction = (None if rows is None else
                          NeighborPrediction(inputs.source_stamp, dt, rows))
            trajectory = (None if rows is None or controls is None else
                          ExecutableTrajectory(inputs.source_stamp, dt,
                                               rows[:, :2], rows[:, 2:4], controls))
            ok = out['ok']
            nominal = out.get('nominal_ok', ok)
            vehicles[name] = VehicleResult(
                nominal_success=nominal, fallback_used=ok and not nominal,
                publication_eligible=ok, reason=out['reason'],
                trajectory=trajectory, prediction=prediction,
                diagnostics={key: value for key, value in out.items() if key not in (
                    'ok', 'nominal_ok', 'reason', 'prediction', 'controls',
                    'position', 'velocity', 'acceleration')})
        return CycleResult(inputs.source_stamp, vehicles, self.diagnostics())

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

    def occupancy_maps(self):
        return [planner.gmap for planner in self.controller.planners.values()
                if planner.gmap.occupied is not None and planner.gmap.unknown is not None]

    def diagnostics(self):
        c = self.controller
        return dict(algorithm='mpc', leader=c.leader, controller_modes=dict(c.modes),
                    assignment=dict(c.assignment), slot_scales=dict(c.slot_scales))

    def close(self):
        self.controller.close()
