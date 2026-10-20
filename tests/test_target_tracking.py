"""External positions stay exact; timestamps determine velocity and freshness."""

import numpy as np
import pytest

from solo_mpc.core.tracking import ExactTargetTracker


def test_exact_samples_preserve_turns_and_derive_velocity_from_timestamps():
    track = ExactTargetTracker()
    for position, stamp, velocity in [([1, 2], 3., [0, 0]),
                                      ([3, 2], 3.5, [4, 0]),
                                      ([3, 1], 3.75, [0, -4])]:
        assert track.observe([*position, .02], stamp)
        np.testing.assert_array_equal(track.position(stamp), position)
        np.testing.assert_array_equal(track.x, [*position, *velocity])
        assert track.observed_at == stamp
    np.testing.assert_allclose(track.trajectory(3.75, 2, .1, 3), [[3, 1], [3, .7], [3, .4]])
    # Predictions cannot renew the feed's freshness, and coasting remains bounded.
    track.position(100.)
    np.testing.assert_array_equal(track.velocity(100.), [0, 0])
    assert track.observed_at == 3.75


@pytest.mark.parametrize('position,stamp', [([9, 9], 2.), ([9, 9], 1.),
                                          ([float('nan'), 1], 3.), ([1], 3.),
                                          ([1, 2], float('nan'))])
def test_invalid_or_nonincreasing_exact_samples_do_not_replace_the_latest(position, stamp):
    track = ExactTargetTracker()
    assert track.observe([1, 2], 2.)
    assert not track.observe(position, stamp)
    np.testing.assert_array_equal(track.x, [1, 2, 0, 0])
    assert track.observed_at == 2.
