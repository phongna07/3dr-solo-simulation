"""Retained plans replayed by simulation time with measured-state agreement.

A plan solved at ``stamp`` may be replayed on a later failed cycle only while
the measured vehicle still agrees with the plan row at the elapsed simulation
time; a solve counter is never used. ``revalidate_retained`` repeats the check
against the measurement at result collection.
"""

from dataclasses import dataclass, replace

import numpy as np

TOLERANCES = ("BACKUP_POSITION_TOLERANCE", "BACKUP_VELOCITY_TOLERANCE", "BACKUP_ACCEL_TOLERANCE")


@dataclass
class StopPlan:
    states: np.ndarray
    controls: np.ndarray
    stamp: float


def stop_tail(plan, state, stamp, cfg):
    """``(plan, index)`` of the row due at ``stamp``, or None: drop the plan.

    A model certificate cannot justify replay after the measured vehicle deviates from it.
    The publisher's existing expiry/hold behavior handles a rejected tail.
    """
    if plan is None:
        return None
    elapsed = (stamp - plan.stamp) / cfg["TIMESTEP"]
    if not np.isfinite(elapsed) or elapsed < -1e-6:
        return None
    index = min(int(np.floor(elapsed + 1e-6)), len(plan.states) - 1)
    measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
    for values, expected, key in zip(measured, np.split(plan.states[index], 3), TOLERANCES):
        if not np.isfinite(values).all() or np.linalg.norm(values - expected) > cfg[key]:
            return None
    return plan, index


def revalidate_retained(result, state, cfg):
    """Reject a replayed plan whose rows no longer match the collection-time measurement."""
    if not result.publication_eligible or not result.diagnostics.get('retained_plan'):
        return result
    prediction = result.prediction
    rows = prediction.states
    velocity_stamp = state.stamp if state.velocity_stamp is None else state.velocity_stamp
    indices = [min(int(max(0., stamp - prediction.source_stamp) /
                       prediction.timestep + 1e-6), len(rows)-1)
               for stamp in (state.stamp, velocity_stamp, velocity_stamp)]
    measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
    expected = [rows[index, 2 * component:2 * component + 2]
                for component, index in enumerate(indices)]
    matches = all(np.isfinite(value).all() and np.linalg.norm(value - target) <= cfg[key]
                  for value, target, key in zip(measured, expected, TOLERANCES))
    return result if matches else replace(
        result, publication_eligible=False, reason='stop_plan_measured_state_changed')
