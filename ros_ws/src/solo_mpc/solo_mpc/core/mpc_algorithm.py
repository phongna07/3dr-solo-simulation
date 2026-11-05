"""Existing isolated MPC workers exposed through the algorithm boundary."""

from .parallel_controller import ParallelTeamController


class MPCAlgorithm(ParallelTeamController):
    def __init__(self, names, cfg, settings):
        super().__init__(
            names, cfg,
            timeout=settings.get('controller_worker_timeout_wall', 1.0),
            startup_timeout=settings.get('controller_worker_startup_timeout_wall', 30.0),
            sequential=settings.get('controller_execution', 'parallel') == 'sequential',
        )

    def occupancy_maps(self):
        return [planner.gmap for planner in self.planners.values()
                if planner.gmap.occupied is not None and planner.gmap.unknown is not None]

    def diagnostics(self):
        return dict(algorithm='mpc', leader=self.leader, controller_modes=self.modes,
                    assignment=self.assignment, slot_scales=self.slot_scales)
