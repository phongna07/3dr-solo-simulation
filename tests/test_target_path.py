from pathlib import Path

import numpy as np
import pytest

from solo_mpc.core.geometry import catmull_rom
from solo_mpc.core.target_path import target_path
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'


def test_big1000_preserves_sprint_speed_on_each_spline_leg():
    scenario, _, cfg = load_settings(PKG / 'scenarios/big1000_5uav.yaml',
                                     PKG / 'config/controller_big1000.yaml')
    path, times = target_path(scenario, cfg)
    _, knots = catmull_rom(scenario['target']['waypoints'], cfg['TAR_SPLINE_DS'])
    np.testing.assert_allclose(path[knots], scenario['target']['waypoints'])
    assert np.all(np.diff(times) > 0)
    speeds = np.asarray(scenario['target']['speeds'])
    assert np.flatnonzero(speeds == 13).tolist() == [5, 7, 10, 17]
    for leg, speed in enumerate(speeds):
        start, end = knots[leg:leg+2]
        lengths = np.linalg.norm(np.diff(path[start:end+1], axis=0), axis=1)
        np.testing.assert_allclose(lengths / np.diff(times[start:end+1]), speed)


def test_default_target_keeps_constant_speed_timing():
    scenario, _, cfg = load_settings(PKG / 'scenarios/small50_3uav.yaml',
                                     PKG / 'config/controller.yaml')
    path, times = target_path(scenario, cfg)
    expected = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    np.testing.assert_allclose(times, expected / scenario['tar_max_speed'])


def test_unsmoothed_route_changes_speed_at_waypoint():
    scenario = dict(tar_max_speed=5, target=dict(
        waypoints=[[0, 0], [10, 0], [10, 26]], speeds=[5, 13]))
    path, times = target_path(scenario, {'TAR_SMOOTH_ENABLE': False})
    np.testing.assert_allclose(path, scenario['target']['waypoints'])
    np.testing.assert_allclose(times, [0, 2, 4])
    np.testing.assert_allclose([np.interp(3, times, path[:, i]) for i in (0, 1)], [10, 13])


@pytest.mark.parametrize('speeds', [[5], [5, 0], [5, -1], [5, float('nan')], [5, float('inf')]])
def test_bad_leg_speeds_fail_during_configuration_loading(tmp_path, speeds):
    import yaml

    scenario = yaml.safe_load((PKG / 'scenarios/small50_3uav.yaml').read_text())
    scenario['target'] = dict(waypoints=[[0, 0], [10, 0], [10, 26]], speeds=speeds)
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(scenario))
    with pytest.raises(ValueError, match='target.speeds'):
        load_settings(path, PKG / 'config/controller.yaml')
