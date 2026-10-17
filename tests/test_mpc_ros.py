"""Construct every ROS adapter before attempting a Gazebo mission."""

from pathlib import Path
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("casadi")
pytest.importorskip("gz.transport13")
from solo_mpc.controller_node import Controller
from solo_mpc.frame_node import Frames
from solo_mpc.perception_node import MarkerDetector
from solo_mpc.setpoint_node import Setpoints
from solo_mpc.settings import load_settings
from solo_mpc.target_node import MovingTarget


@pytest.mark.parametrize(
    "node_type", [Frames, MarkerDetector, MovingTarget, Controller, Setpoints]
)
def test_nodes_construct_without_conflicting_with_rclpy_apis(node_type, tmp_path):
    root = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"
    scenario, settings, cfg = load_settings(
        root / "scenarios/small50_3uav.yaml", root / "config/controller.yaml"
    )
    args = SimpleNamespace(
        names=["test_uav"],
        settings_data=settings,
        cfg=cfg,
        scenario_data=scenario,
        run_dir=str(tmp_path),
        world="test_world",
    )
    rclpy.init()
    node = None
    try:
        node = node_type(args)
        assert node.get_name().startswith("mpc_")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def node_args(tmp_path):
    root = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"
    scenario, settings, cfg = load_settings(
        root / "scenarios/small50_3uav.yaml", root / "config/controller.yaml"
    )
    return SimpleNamespace(
        names=["test_uav"],
        settings_data=settings,
        cfg=cfg,
        scenario_data=scenario,
        run_dir=str(tmp_path),
        world="test_world",
    )


def test_per_vehicle_camera_subscribes_only_to_its_own_topics(tmp_path):
    args = node_args(tmp_path)
    args.names = ["alpha", "bravo", "charlie"]
    args.vehicle = "bravo"
    rclpy.init()
    node = MarkerDetector(args)
    try:
        assert set(node.history) == {"bravo"}
        assert set(node.publishers_keep) == {"bravo"}
        assert {s.topic_name for s in node.subscriptions_keep} == {
            "/bravo/mpc/pose", "/bravo/camera/image_raw", "/bravo/camera/camera_info",
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('fail', [False, True])
def test_run_node_records_constructor_success_or_failure(tmp_path, monkeypatch, fail):
    import json
    from solo_mpc import ros_utils

    args = node_args(tmp_path)
    args.startup_component = 'setpoints'
    monkeypatch.setattr(ros_utils, 'arguments', lambda _: (args, []))
    path = tmp_path / 'startup-components/setpoints.json'

    def construct(options):
        assert json.loads(path.read_text())['state'] == 'initializing'
        if fail:
            raise RuntimeError('constructor failed')
        return Setpoints(options)

    def spin(node):
        assert json.loads(path.read_text())['state'] == 'initialized'
        raise KeyboardInterrupt

    monkeypatch.setattr(rclpy, 'spin', spin)
    if fail:
        with pytest.raises(RuntimeError, match='constructor failed'):
            ros_utils.run_node(construct, 'Startup test')
        assert json.loads(path.read_text())['state'] == 'failed'
        assert 'constructor failed' in json.loads(path.read_text())['detail']
    else:
        ros_utils.run_node(construct, 'Startup test')
    assert not rclpy.ok()


def test_compass_error_does_not_rotate_enu_position_axes(tmp_path):
    import numpy as np
    from mavros_msgs.msg import State
    from scipy.spatial.transform import Rotation
    from solo_mpc.ros_utils import pose_message

    rclpy.init()
    node = Frames(node_args(tmp_path))
    try:
        node.state("test_uav", State(connected=True, armed=False))
        stamp = node.get_clock().now().to_msg()
        truth = pose_message(
            [8.0, 2.0, 0.195],
            stamp,
            Rotation.from_euler("z", 90, degrees=True).as_quat(),
        )
        # A ten-degree compass bias must not rotate a twenty-metre displacement.
        local = pose_message(
            [0.0, 0.0, 0.0], stamp, Rotation.from_euler("z", 80, degrees=True).as_quat()
        )
        node.ground_pose("test_uav", truth)
        for _ in range(10):
            node.local_pose("test_uav", local)
        rot, offset = node.transforms["test_uav"]
        assert np.allclose(rot.apply([20.0, 0.0, 3.0]) + offset, [28.0, 2.0, 3.195])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_raw_setpoints_transform_enu_and_expire_to_zero_velocity(tmp_path):
    import time

    import numpy as np
    from mavros_msgs.msg import PositionTarget, State
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message

    rclpy.init()
    node = Setpoints(node_args(tmp_path))
    try:
        name = "test_uav"
        sent = []
        node.publishers_keep[name] = SimpleNamespace(publish=sent.append)
        node.align[name] = {"yaw": np.pi / 2, "translation": [2.0, 3.0, 0.2]}
        node.enabled = True
        node.state(name, State(connected=True, armed=True, mode="GUIDED"))
        stamp = node.get_clock().now().to_msg()
        node.pose(name, pose_message([10.0, 5.0, 3.0], stamp))
        reference = PositionTarget()
        reference.header.frame_id = "world"
        reference.header.stamp = stamp
        reference.position.x, reference.position.y, reference.position.z = (
            10.0,
            5.0,
            3.0,
        )
        reference.velocity.x, reference.velocity.y = 0.4, 0.2
        reference.acceleration_or_force.x, reference.acceleration_or_force.y = 0.2, -0.1
        node.reference(name, reference)
        node.tick()
        command = sent[-1]
        assert np.allclose(
            [command.position.x, command.position.y, command.position.z],
            [2.0, -8.0, 2.8],
        )
        assert np.allclose([command.velocity.x, command.velocity.y], [0.2, -0.4])
        assert np.allclose(
            [command.acceleration_or_force.x, command.acceleration_or_force.y],
            [-0.1, -0.2],
        )
        assert (
            command.type_mask
            == PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        # A slow simulator stretches low-rate FCU messages in wall time.
        state, _ = node.states[name]
        node.states[name] = (state, time.monotonic() - 5.0)
        node.tick()
        assert sent[-1].type_mask == command.type_mask
        node.state_stamps[name] -= 3.0
        node.tick()
        assert sent[-1].type_mask & PositionTarget.IGNORE_PX
        node.state(name, state)
        reference.header.stamp = Time(
            nanoseconds=node.get_clock().now().nanoseconds - 1_000_000_000
        ).to_msg()
        node.tick()
        expired = sent[-1]
        assert expired.type_mask & PositionTarget.IGNORE_PX
        assert np.allclose(
            [expired.velocity.x, expired.velocity.y, expired.velocity.z],
            [0.0, 0.0, 0.0],
        )
        pose, _ = node.poses[name]
        node.poses[name] = (pose, time.monotonic() - 5.0)
        node.tick()
        assert sent[-1].velocity.z == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('projection,empty,reason', [
    ('extruded', False, 'ready'),
    ('horizontal', False, 'insufficient_valid_scan'),
    ('extruded', True, 'insufficient_valid_scan'),
])
def test_five_uav_controller_projects_tilted_scans_and_logs_why_it_stops(
        tmp_path, monkeypatch, projection, empty, reason):
    import json
    import numpy as np
    from mavros_msgs.msg import State
    from scipy.spatial.transform import Rotation
    from sensor_msgs.msg import LaserScan
    from solo_mpc import controller_node
    from solo_mpc.ros_utils import seconds

    root = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'
    scenario, settings, cfg = load_settings(root / 'scenarios/big1000_5uav.yaml',
                                           root / 'config/controller_big1000.yaml')
    settings['controller_execution'] = 'sequential'
    settings['scan_projection_mode'] = projection
    names = [f'uav{i}' for i in range(1, 6)]
    calls = []

    def step(states, scans, stamp, seers):
        calls.append(scans)
        return {n: dict(ok=True, position=s.position[:2], velocity=np.zeros(2),
                        acceleration=np.zeros(2), reason='tracking',
                        prediction=np.tile(np.r_[s.position[:2], np.zeros(4)], (21, 1)))
                for n, s in states.items()}

    team = SimpleNamespace(track=SimpleNamespace(observed_at=0., x=np.zeros(4)),
                           leader='uav3', step=step)
    monkeypatch.setattr(controller_node, 'TeamController', lambda *args: team)
    args = SimpleNamespace(names=names, settings_data=settings, cfg=cfg,
                           scenario_data=scenario, run_dir=str(tmp_path))
    rclpy.init()
    node = Controller(args)
    try:
        records = []
        node.status_pub = SimpleNamespace(publish=lambda msg: records.append(json.loads(msg.data)))
        stamp = node.get_clock().now().to_msg()
        at = seconds(stamp)
        team.track.observed_at = at
        for n, start in zip(names, scenario['starts']):
            p = np.asarray(start, float)
            q = Rotation.from_euler('y', 5, degrees=True).as_quat()
            node.poses[n] = (p, q, at)
            node.histories[n].add(at, p, q)
            node.velocities[n] = (np.zeros(3), at)
            node.fcu[n] = State(connected=True, armed=True, mode='GUIDED')
            scan = LaserScan(angle_min=-np.pi, angle_increment=2*np.pi/180,
                             range_min=.5, range_max=50.)
            scan.header.stamp = stamp
            scan.ranges = [] if empty else [float('inf')] * 180
            node.scans[n] = scan
        node.tick()
        record = records[-1]
        assert record['reason'] == reason
        if reason == 'ready':
            assert len(calls) == 1 and set(calls[0]) == set(names)
            assert len(record['scan_diagnostics']) == 5
            assert all(d['retained_fraction'] == 1. for d in record['scan_diagnostics'].values())
        else:
            assert not calls
            assert record['scan_diagnostics']['uav1']['retained_rays'] == (0 if empty else 26)
        assert record['scan_diagnostics']['uav1']['pitch_deg'] == pytest.approx(5.)
        assert record['scan_diagnostics']['uav1']['projection_mode'] == projection
    finally:
        node.destroy_node()
        rclpy.shutdown()
