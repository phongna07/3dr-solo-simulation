"""Tilted long-range scans retain only observed space inside extruded obstacles."""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from solo_mpc.core.mapping import SensorGrid
from solo_mpc.core.sensors import project_scan
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'


def scan(ranges, pitch=5, roll=0, vertical_bounds=None, position=(100, 100, 30)):
    return project_scan(ranges, -np.pi, 2*np.pi/len(ranges), .5, 50., position,
                        Rotation.from_euler('xyz', [roll, pitch, 0], degrees=True).as_quat(),
                        1., vertical_bounds=vertical_bounds)


def test_long_range_horizontal_gate_reproduces_reported_scan_failure():
    ranges = np.full(180, np.inf)
    assert len(scan(ranges, pitch=0).endpoints) == 180
    assert len(scan(ranges, pitch=2).endpoints) == 70  # below 70% controller gate
    assert len(scan(ranges, pitch=5).endpoints) == 26


@pytest.mark.parametrize('roll,pitch', [(0, 2), (0, 5), (10, 15), (0, 27), (27, 0)])
def test_extruded_projection_clips_tilted_no_returns_to_height_band(roll, pitch):
    rays = scan(np.full(180, np.inf), pitch, roll, (.3, 39.5))
    assert len(rays.endpoints) == 180
    assert not rays.hits.any()
    assert np.min(rays.endpoints[:, 2]) >= .3 - 1e-10
    assert np.max(rays.endpoints[:, 2]) <= 39.5 + 1e-10
    # Ray clipping never extends a measured ray or invents a new direction.
    original = Rotation.from_euler('xyz', [roll, pitch, 0], degrees=True).apply(
        np.c_[50*np.cos(np.linspace(-np.pi, np.pi, 180, endpoint=False)),
                 50*np.sin(np.linspace(-np.pi, np.pi, 180, endpoint=False)), np.zeros(180)])
    delta = rays.endpoints - rays.origin
    assert np.all(np.linalg.norm(delta, axis=1) <= 50. + 1e-10)
    np.testing.assert_allclose(np.cross(delta, original), 0, atol=1e-10)


def test_extruded_projection_does_not_turn_missing_rays_into_clear_space():
    rays = scan([np.nan, 0., -.5, .1, -np.inf, np.inf, 10.], vertical_bounds=(.3, 39.5))
    assert len(rays.endpoints) == 2
    assert rays.hits.tolist() == [False, True]


def test_finite_obstacle_hit_inside_band_survives_tilt_projection():
    rays = project_scan([10.], 0., 1., .5, 50., [100, 100, 30],
                        Rotation.from_euler('y', 20, degrees=True).as_quat(),
                        1., vertical_bounds=(.3, 39.5))
    assert rays.hits.tolist() == [True]
    np.testing.assert_allclose(rays.endpoints[0],
                               [100 + 10*np.cos(np.deg2rad(20)), 100,
                                30 - 10*np.sin(np.deg2rad(20))])
    grid = SensorGrid((0, 0, 200, 200), resolution=1., inflate_radius=.5)
    grid.integrate(rays)
    x, y = grid.world_to_grid(*rays.endpoints[0, :2])
    assert grid.L[y, x] > 0


@pytest.mark.parametrize('angle', [0., np.pi])
def test_rays_leaving_obstacle_band_leave_remainder_unknown(angle):
    # One finite return on ground/beyond obstacle tops: retain the free prefix,
    # but do not mark the band boundary as an obstacle or clear beyond it.
    rays = project_scan([50.], angle, 1., .5, 50., [100, 100, 30],
                        Rotation.from_euler('y', 45, degrees=True).as_quat(),
                        1., vertical_bounds=(.3, 39.5))
    assert rays.hits.tolist() == [False]
    grid = SensorGrid((0, 0, 200, 200), resolution=1., inflate_radius=.5)
    grid.integrate(rays)
    direction = (rays.endpoints[0, :2] - rays.origin[:2])
    length = np.linalg.norm(direction)
    direction /= length
    x, y = grid.world_to_grid(*(rays.origin[:2] + direction * 3))
    assert grid.L[y, x] < 0
    x, y = grid.world_to_grid(*(rays.endpoints[0, :2] + direction * 3))
    assert grid.L[y, x] == 0


@pytest.mark.parametrize('altitude', [.1, .3, 39.5, 45.])
def test_origin_outside_band_cannot_establish_free_space(altitude):
    rays = scan(np.full(180, np.inf), vertical_bounds=(.3, 39.5),
                position=(100, 100, altitude))
    assert len(rays.endpoints) == 0


def test_large_configuration_selects_extruded_projection_and_small_keeps_default():
    _, big, _ = load_settings(PKG / 'scenarios/big1000_5uav.yaml',
                              PKG / 'config/controller_big1000.yaml')
    _, small, _ = load_settings(PKG / 'scenarios/small50_3uav.yaml',
                                PKG / 'config/controller.yaml')
    assert big['scan_projection_mode'] == 'extruded'
    assert small.get('scan_projection_mode', 'horizontal') == 'horizontal'


def test_invalid_projection_mode_fails_before_launch(tmp_path):
    import yaml
    settings = yaml.safe_load((PKG / 'config/controller_big1000.yaml').read_text())
    settings['scan_projection_mode'] = 'unknown'
    path = tmp_path / 'controller.yaml'
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError, match='scan_projection_mode'):
        load_settings(PKG / 'scenarios/big1000_5uav.yaml', path)
