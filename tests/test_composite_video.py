"""Decode combined recordings to check layout and simulation-time alignment."""

import json

import cv2
import numpy as np
import pytest

from solo_mpc.core.composite_video import CompositeVideo, overlay_views


def image(color, size=(40, 80)):
    return np.full((*size, 3), color, np.uint8)


def decode(path):
    capture = cv2.VideoCapture(str(path))
    try:
        assert capture.isOpened()
        fps = capture.get(cv2.CAP_PROP_FPS)
        frames = []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
        return fps, frames
    finally:
        capture.release()


def test_combined_video_aligns_delayed_feeds_and_preserves_elapsed_simulation_time(tmp_path):
    path = tmp_path / 'recorded-mission.mp4'
    timestamps = tmp_path / 'telemetry/recorded-mission.timestamps.jsonl'
    recorder = CompositeVideo(path, 20, ['alpha', 'bravo', 'charlie'],
                              timestamps_path=timestamps)
    background = image((80, 80, 80), (360, 640))
    red, green, blue = (0, 0, 240), (0, 240, 0), (240, 0, 0)
    # Different start times and rates, with FOV callbacks delivered after overhead.
    recorder.write('overhead', background, 10_000_000_000)
    recorder.write('alpha', image(red), 10_000_000_000)
    recorder.write('bravo', image(green), 10_000_000_000)
    recorder.write('charlie', image(blue), 10_050_000_000)
    assert recorder.video.frames == 1
    # A dropped overhead frame must hold the background, not shorten playback.
    recorder.write('overhead', image((120, 120, 120), (360, 640)), 10_100_000_000)
    recorder.write('alpha', image(blue), 10_100_000_000)
    recorder.write('charlie', image(red), 10_100_000_000)
    assert recorder.video.frames == 1  # Bravo's delayed frame is still pending.
    recorder.write('bravo', image(red), 10_100_000_000)
    recorder.close()
    recorder.close()

    assert path.read_bytes()[4:8] == b'ftyp'
    fps, frames = decode(path)
    assert fps == 20 and len(frames) == 3
    assert all(frame.shape == background.shape for frame in frames)
    assert np.allclose([frame[180, 20] for frame in frames],
                       [(80, 80, 80), (80, 80, 80), (120, 120, 120)], atol=6)
    # No future image is used: alpha holds red, and charlie starts black.
    for point, colors in [((80, 570), [red, red, blue]),
                          ((195, 570), [green, green, red]),
                          ((310, 570), [(24, 24, 24), blue, red])]:
        assert np.allclose([frame[point] for frame in frames], colors, atol=8)
    stamps = [json.loads(row)['stamp_ns'] for row in
              timestamps.read_text().splitlines()]
    assert stamps == [10_000_000_000, 10_050_000_000, 10_100_000_000]
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        'recorded-mission.mp4', 'telemetry',
    ]


@pytest.mark.parametrize('count', [1, 3, 6, 12])
def test_overlays_fit_every_vehicle_on_the_right_and_preserve_image_aspect(count):
    background = image((70, 70, 70), (720, 1280))
    colors = [(20 + i * 15, 210, 90) for i in range(count)]
    result = overlay_views(background, {f'uav{i}': image(color)
                                        for i, color in enumerate(colors)})
    assert result.shape == background.shape
    assert np.array_equal(result[:, :960], background[:, :960])
    previous_y = -1
    for color in colors:
        ys, xs = np.where(np.all(result == color, axis=2))
        assert len(xs) > 0
        assert ys.min() > previous_y
        previous_y = ys.max()
        # The source is twice as wide as tall; no stretching to square panels.
        assert (xs.max() - xs.min() + 1) / (ys.max() - ys.min() + 1) == pytest.approx(2, abs=0.1)


def test_missing_camera_times_out_even_with_paused_simulation_and_flushes_tail(tmp_path, monkeypatch):
    monkeypatch.setattr('solo_mpc.core.composite_video.time.monotonic', lambda: 100.0)
    recorder = CompositeVideo(tmp_path / 'recorded-mission.mp4', 20, ['missing'])
    background = image((90, 90, 90), (360, 640))
    recorder.write('overhead', background, 1_000_000_000)
    recorder.flush(now=101.9)
    assert recorder.video.frames == 0
    recorder.flush(now=102.1)
    assert recorder.video.frames == 1
    recorder.write('overhead', background, 1_100_000_000)
    recorder.close()
    _, frames = decode(recorder.video.path)
    assert len(frames) == 3
    assert np.allclose(frames[-1][80, 570], (24, 24, 24), atol=6)


def test_simulation_buffer_limits_missing_camera_delay_and_ignores_old_samples(tmp_path):
    recorder = CompositeVideo(tmp_path / 'recorded-mission.mp4', 20, ['missing'])
    background = image((90, 90, 90), (360, 640))
    recorder.write('overhead', background, 1_000_000_000)
    recorder.write('overhead', background, 1_500_000_000)
    assert recorder.video.frames == 1
    recorder.write('overhead', background, 1_500_000_000)
    recorder.write('overhead', background, 1_200_000_000)
    recorder.close()
    assert len(decode(recorder.video.path)[1]) == 11


def test_no_overhead_creates_no_output_and_fov_buffer_is_bounded(tmp_path):
    recorder = CompositeVideo(tmp_path / 'recorded-mission.mp4', 20, ['alpha'])
    for i in range(1000):
        recorder.write('alpha', image((90, 90, 90)), i)
    assert len(recorder.pending['alpha']) <= 256
    recorder.close()
    assert not list(tmp_path.iterdir())


def test_eight_fps_fovs_stay_aligned_with_twenty_fps_overhead_despite_jitter(tmp_path):
    path = tmp_path / 'recorded-mission.mp4'
    names = ['alpha', 'bravo', 'charlie']
    starts = [0, 25_000_000, 75_000_000]
    delays = [30_000_000, 10_000_000, 20_000_000]
    colors = [(0, 0, 240), (0, 240, 0), (240, 0, 0)]
    events = [(i * 50_000_000, 'overhead', i * 50_000_000,
               image((80, 80, 80), (360, 640))) for i in range(21) if i != 7]
    for name, start, delay in zip(names, starts, delays):
        for i in range(10):
            stamp = start + i * 125_000_000
            events.append((stamp + delay, name, stamp, image(colors[i % 3])))
    recorder = CompositeVideo(path, 20, names)
    for _, name, stamp, frame in sorted(events, key=lambda event: event[0]):
        recorder.write(name, frame, stamp)
    recorder.close()
    _, frames = decode(path)
    assert len(frames) == 21  # A missing overhead frame did not compress time.
    for i, frame in enumerate(frames):
        for start, y in zip(starts, [80, 195, 310]):
            sample = (i * 50_000_000 - start) // 125_000_000
            expected = (24, 24, 24) if sample < 0 else colors[sample % 3]
            assert np.allclose(frame[y, 570], expected, atol=8)


def test_shutdown_closes_writer_even_when_composition_fails(tmp_path, monkeypatch):
    recorder = CompositeVideo(tmp_path / 'recorded-mission.mp4', 20, ['missing'])
    recorder.write('overhead', image((90, 90, 90), (360, 640)), 1_000_000_000)

    def fail(*args):
        raise RuntimeError('composition failed')

    monkeypatch.setattr('solo_mpc.core.composite_video.overlay_views', fail)
    with pytest.raises(RuntimeError, match='composition failed'):
        recorder.close()
    assert recorder.video.closed
