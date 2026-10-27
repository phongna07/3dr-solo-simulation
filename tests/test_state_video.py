"""Check instrument meaning, timestamp isolation, and the encoded layout."""

import math

import cv2
import numpy as np
import pytest

from solo_mpc.core.composite_video import CompositeVideo
from solo_mpc.core.occupancy_video import PANEL, overlay_occupancy
from solo_mpc.core.state_video import (
    FlightStateHistory, overlay_states, render_states, vector_tip,
)


def test_state_sampling_never_leaks_future_measurements_or_commands_and_expires():
    history = FlightStateHistory()
    history.write('pose', [1, 2, 3, 0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)], 10**9)
    history.write('velocity', [0, 0, 0], 10**9)
    history.write('velocity', [0.2, -0.1, 0.05], 1_100_000_000)
    history.write('command', [-1, 0, 0], 1_200_000_000)
    history.write('pose', [9, 9, 9, 0, 0, 0, 1], 1_300_000_000)
    state = history.sample(1_100_000_000)
    np.testing.assert_array_equal(state['position'], [1, 2, 3])
    np.testing.assert_allclose(state['acceleration'], [2, -1, 0.5])
    assert state['yaw'] == pytest.approx(math.pi / 2)
    assert state['command'] is None
    assert history.sample(900_000_000)['status'] == 'WAITING'
    assert history.sample(1_000_000_000)['acceleration'] is None
    assert history.sample(2_300_000_001)['status'] == 'STALE'
    assert all(value is None for key, value in history.sample(2_300_000_001).items()
               if key != 'status')


def test_estimated_acceleration_filters_noise_and_resets_after_gaps():
    history = FlightStateHistory()
    history.write('velocity', [0, 0, 0], 0)
    history.write('velocity', [0.1, 0, 0], 100_000_000)
    history.write('velocity', [0.1, 0, 0], 200_000_000)
    np.testing.assert_allclose(history.sample(200_000_000)['acceleration'], [0.7, 0, 0])
    # Invalid/old deliveries cannot corrupt the next derivative.
    history.write('velocity', [99, 99, 99], 100_000_000)
    history.write('velocity', [np.nan, 0, 0], 300_000_000)
    history.write('velocity', [10, 0, 0], 2_000_000_000)
    assert history.sample(2_000_000_000)['acceleration'] is None
    for i in range(1000):
        history.write('command', [1, 0, 0], i)
    assert len(history.samples['command']) == 256


def test_instrument_arrows_use_world_axes_fixed_scales_and_cap_large_vectors():
    assert vector_tip((80, 80), [1, 0, 5], 2, 32) == (96, 80)
    assert vector_tip((80, 80), [0, 2, 0], 2, 32) == (80, 48)
    assert vector_tip((80, 80), [-20, 0, 0], 2, 32) == (48, 80)
    assert vector_tip((80, 80), [0, 0, -1], 2, 32) == (80, 80)


@pytest.mark.parametrize('count', [1, 3, 6, 12])
def test_map_and_state_panels_fit_without_overlapping_or_covering_center_and_fovs(count):
    frame = np.full((1080, 1920, 3), 90, np.uint8)
    # A tall map is the worst case for overlap.
    map_panel = np.full((1000, 560, 3), 180, np.uint8)
    overlay_occupancy(frame, map_panel, max_height=0.5)
    upper = frame.copy()
    states = {f'uav{i}': FlightStateHistory().sample(0) for i in range(count)}
    overlay_states(frame, render_states(states, 0))
    assert np.array_equal(frame[:540], upper[:540])
    assert np.array_equal(frame[:, 576:], upper[:, 576:])
    assert not np.array_equal(frame[600:1068, 12:300], upper[600:1068, 12:300])


def test_encoded_state_panel_uses_video_time_without_blocking_on_telemetry(tmp_path, monkeypatch):
    from solo_mpc.core import composite_video

    samples = []
    render = composite_video.render_states

    def capture(states, stamp):
        samples.append((stamp, states['alpha']))
        return render(states, stamp)

    monkeypatch.setattr(composite_video, 'render_states', capture)
    recorder = CompositeVideo(tmp_path / 'combined.avi', 20, ['alpha'],
                              occupancy=True, telemetry=True)
    # A command arriving before its matching video frame cannot travel back in time.
    recorder.write_state('alpha', 'command', [1, 0, 0], 1_100_000_000)
    background = np.full((1080, 1920, 3), 90, np.uint8)
    recorder.write('overhead', background, 10**9)
    recorder.write('alpha', background, 10**9)
    assert recorder.video.frames == 1  # Missing pose/velocity never block export.
    recorder.write('overhead', background, 1_100_000_000)
    recorder.write('alpha', background, 1_100_000_000)
    recorder.close()
    assert [at for at, _ in samples] == [10**9, 1_050_000_000, 1_100_000_000]
    assert all(state['command'] is None for _, state in samples[:2])
    np.testing.assert_array_equal(samples[-1][1]['command'], [1, 0, 0])
    video = cv2.VideoCapture(str(recorder.video.path))
    try:
        assert video.get(cv2.CAP_PROP_FRAME_COUNT) == 3
        assert video.get(cv2.CAP_PROP_FPS) == 20
        ok, frame = video.read()
        assert ok and frame.shape == background.shape
        assert np.allclose(frame[1060, 20], PANEL, atol=8)
        assert np.allclose(frame[540, 960], 90, atol=6)
    finally:
        video.release()
