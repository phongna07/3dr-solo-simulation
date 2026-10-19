"""Offline contracts for the field scenario; no Gazebo or FCU is started."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from solo_bringup.configuration import load_config
from solo_mpc.core.geometry import catmull_rom, timed_target_path
from solo_mpc.settings import load_settings, scenario_fleet

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros_ws/src/solo_mpc"
SCENARIO = PKG / "scenarios/big1000_5uav.yaml"
SETTINGS = PKG / "config/controller_big1000.yaml"


def test_large_configuration_preserves_map_and_uses_five_independent_fcu_links():
    scenario, settings, cfg = load_settings(SCENARIO, SETTINGS)
    fleet = scenario_fleet(load_config(ROOT / "config/fleet_5uav.yaml"), scenario)
    assert len(fleet["uavs"]) == len(scenario["starts"]) == 5
    assert [u["sysid"] for u in fleet["uavs"]] == [1, 2, 3, 4, 5]
    assert [u["fcu_url"] for u in fleet["uavs"]] == [f"tcp://127.0.0.1:{5760 + 10*i}" for i in range(5)]
    assert [u["json_port"] for u in fleet["uavs"]] == [9002, 9012, 9022, 9032, 9042]
    assert cfg["WORLD_BOUNDS"] == (0, 0, 1000, 1000)
    assert cfg["VMAX"] == 10 and cfg["UMAX"] == 5 and cfg["HORIZON_LENGTH"] == 20
    assert cfg["ROBOT_RADIUS"] == 0.5 and cfg["INFLATE_RADIUS"] == 1.5
    assert len(cfg["GRID_CELLS"]) == 4
    assert settings["controller_execution"] == "parallel"
    assert settings["flight_altitude"] == 30
    assert settings["obstacle_height"] > settings["flight_altitude"] + settings["scan_height_tolerance"]
    assert len(scenario["obstacles"]["rects"]) == 60
    assert len(scenario["obstacles"]["circles"]) == 48
    assert len(scenario["target"]["waypoints"]) == 33
    assert np.flatnonzero(np.asarray(scenario["target"]["speeds"]) == 13).tolist() == [4, 7, 11, 14, 22]
    half_side = (settings["flight_altitude"] - 0.04 - settings["target_height"] - 0.02) * np.tan(settings["camera_hfov"] / 2)
    assert half_side == pytest.approx(scenario["viewing_radius"], rel=0.005)
    marker_pixels = settings["camera_width"] * cfg["TAR_RADIUS"] / half_side
    assert marker_pixels**2 > settings["marker_min_pixels"]
    # At least one real downward camera can acquire the stationary first target.
    first = np.asarray(scenario["target"]["waypoints"][0])
    assert np.flatnonzero(np.max(abs(np.asarray(scenario["starts"])[:, :2] - first), axis=1) < half_side).tolist() == [3]
    assert settings["arrival_fleet_radius"] > cfg["VIEWING_RADIUS"] * (cfg["SLOT_SPACING_RATIO"] + 1)


def test_target_sprint_speeds_follow_smoothed_arc_length_on_the_correct_legs():
    scenario, _, cfg = load_settings(SCENARIO, SETTINGS)
    path, times = timed_target_path(scenario, cfg)
    _, knots = catmull_rom(scenario["target"]["waypoints"], cfg["TAR_SPLINE_DS"])
    np.testing.assert_allclose(path[knots], scenario["target"]["waypoints"])
    assert np.all(np.diff(times) > 0)
    for i, speed in enumerate(scenario["target"]["speeds"]):
        start, end = knots[i:i+2]
        distances = np.linalg.norm(np.diff(path[start:end+1], axis=0), axis=1)
        np.testing.assert_allclose(distances / np.diff(times[start:end+1]), speed)


def test_field_lidar_retains_rays_when_banked_for_configured_acceleration():
    from scipy.spatial.transform import Rotation
    from solo_mpc.core.sensors import project_scan

    _, settings, cfg = load_settings(SCENARIO, SETTINGS)
    tilt = np.arctan(np.sqrt(2) * cfg["UMAX"] / 9.81)
    count = settings["lidar_samples"]
    scan = project_scan(np.full(count, np.inf), -np.pi, 2*np.pi/count,
                        cfg["LIDAR_RANGE_MIN"], cfg["SENSING_RADIUS"],
                        [500, 500, settings["flight_altitude"] + 0.08],
                        Rotation.from_euler("y", tilt).as_quat(), 0.,
                        height_tolerance=settings["scan_height_tolerance"])
    assert len(scan.endpoints) == count
    assert np.all(scan.endpoints[:, 2] < settings["obstacle_height"])


@pytest.mark.parametrize("smooth", [False, True])
def test_target_without_leg_speeds_keeps_constant_speed_behavior(smooth):
    scenario, _, cfg = load_settings(PKG / "scenarios/small50_3uav.yaml", PKG / "config/controller.yaml")
    cfg["TAR_SMOOTH_ENABLE"] = smooth
    path, times = timed_target_path(scenario, cfg)
    expected = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))] / scenario["tar_max_speed"]
    np.testing.assert_allclose(times, expected)
    if not smooth:
        np.testing.assert_array_equal(path, scenario["target"]["waypoints"])


@pytest.mark.parametrize("speeds", [[5], [5] * 31 + [0], [5] * 31 + [float("nan")]])
def test_invalid_leg_speeds_are_rejected_before_launch(tmp_path, speeds):
    scenario = yaml.safe_load(SCENARIO.read_text())
    scenario["target"]["speeds"] = speeds
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario))
    with pytest.raises(ValueError, match="target.speeds"):
        load_settings(path, SETTINGS)


def test_small_map_settings_cannot_silently_fly_large_scenario_at_three_metres():
    with pytest.raises(ValueError, match="flight_altitude"):
        load_settings(SCENARIO, PKG / "config/controller.yaml")
