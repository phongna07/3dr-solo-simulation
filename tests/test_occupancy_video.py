"""Check fleet-map meaning, world orientation, and the encoded overlay timeline."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from solo_mpc.core.composite_video import CompositeVideo
from solo_mpc.core.occupancy_video import (
    FREE, OCCUPIED, PANEL, UNKNOWN, fuse_occupancy, render_occupancy,
)


def test_fleet_union_preserves_unknown_and_obstacle_conflicts_without_mutating_maps():
    def snapshot(cells):
        cells = np.asarray(cells)
        return SimpleNamespace(occupied=cells == 100, unknown=cells == -1)

    alpha = snapshot([[-1, 0, 100], [-1, -1, 0]])
    bravo = snapshot([[-1, -1, 0], [100, 0, -1]])
    before = [(m.occupied.copy(), m.unknown.copy()) for m in (alpha, bravo)]
    cells = fuse_occupancy([alpha, bravo])
    np.testing.assert_array_equal(cells, [[-1, 0, 100], [100, 0, 0]])
    assert cells.dtype == np.int8
    for m, (occupied, unknown) in zip((alpha, bravo), before):
        np.testing.assert_array_equal(m.occupied, occupied)
        np.testing.assert_array_equal(m.unknown, unknown)


def test_rectangular_grid_uses_world_y_up_and_distinguishes_all_three_states():
    cells = np.full((20, 40), -1, np.int8)
    cells[:10, :20] = 0  # Southwest, with a negative world origin.
    cells[10:, 20:] = 100  # Northeast.
    panel = render_occupancy(cells, 0.5, (-10., -5.), {}, ['alpha'], 12_000_000_000)
    assert panel.shape[1] == 560
    np.testing.assert_array_equal(panel[140, 100], UNKNOWN)
    np.testing.assert_array_equal(panel[140, 420], OCCUPIED)
    np.testing.assert_array_equal(panel[280, 100], FREE)
    # X and Y retain the same metric scale despite a non-square grid.
    ys, xs = np.where(np.all(panel[82:342] == FREE, axis=2))
    assert (xs.max() - xs.min() + 1) / (ys.max() - ys.min() + 1) == pytest.approx(2, abs=0.03)


def test_encoded_map_never_uses_future_scan_and_does_not_block_missing_maps(tmp_path):
    path = tmp_path / 'combined.avi'
    recorder = CompositeVideo(path, 20, ['alpha'], occupancy=True)
    background = np.full((360, 640, 3), 90, np.uint8)
    future = np.full((100, 100, 3), (40, 70, 210), np.uint8)
    # Map delivery precedes the matching overhead tick, but must not leak back.
    recorder.write('occupancy', future, 1_100_000_000)
    recorder.write('overhead', background, 1_000_000_000)
    recorder.write('alpha', background, 1_000_000_000)
    assert recorder.video.frames == 1  # A missing map never holds the cameras.
    recorder.write('overhead', background, 1_100_000_000)
    recorder.write('alpha', background, 1_100_000_000)
    recorder.write('overhead', background, 1_200_000_000)
    recorder.write('alpha', background, 1_200_000_000)
    recorder.close()
    capture = cv2.VideoCapture(str(path))
    try:
        assert capture.get(cv2.CAP_PROP_FPS) == 20
        frames = []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
        assert len(frames) == 5
        assert np.allclose([f[60, 50] for f in frames[:2]], PANEL, atol=8)
        assert np.allclose([f[60, 50] for f in frames[2:]], (40, 70, 210), atol=8)
        assert all(np.allclose(f[180, 320], 90, atol=6) for f in frames)
    finally:
        capture.release()
