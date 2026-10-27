"""Send camera images over ROS and verify each UAV's finalized video."""

import json
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from solo_mpc.camera_recorder import CameraRecorder


@pytest.mark.parametrize("overhead", [False, True])
def test_ros_camera_feeds_are_recorded_separately_and_closed_on_shutdown(tmp_path, overhead):
    names = ["overhead"] if overhead else [
        "camera_test_alpha", "camera_test_bravo", "camera_test_charlie",
    ]
    topics = ["/overhead/camera/image_raw"] if overhead else [
        f"/{name}/camera/image_raw" for name in names
    ]
    colors = [(0, 0, 255)] if overhead else [(0, 0, 255), (0, 255, 0), (255, 0, 255)]
    width, height, fps = (1920, 1080, 20.0) if overhead else (160, 160, 8.0)
    rclpy.init()
    source = rclpy.create_node("camera_test_source")
    vehicles = ['camera_test_alpha', 'camera_test_bravo', 'camera_test_charlie']
    recorder = CameraRecorder(names, tmp_path, fps, overhead=overhead, vehicle_names=vehicles)
    try:
        if overhead:
            assert not recorder.image_subscriptions
        else:
            assert set(recorder.image_subscriptions) == set(names)
        for vehicle in vehicles:
            recorder.state(vehicle, State(connected=True, armed=False))
            pose = PoseStamped()
            pose.pose.position.z = 10.0
            recorder.pose(vehicle, pose)
            recorder.state(vehicle, State(connected=True, armed=True))
            pose.pose.position.z = 10.25
            recorder.pose(vehicle, pose)
        assert {s.topic_name for s in recorder.subscriptions_keep if s.msg_type is Image} == set(topics)
        publishers = [source.create_publisher(
            Image, topic, qos_profile_sensor_data,
        ) for topic in topics]
        deadline = time.monotonic() + 10
        while any(pub.get_subscription_count() == 0 for pub in publishers):
            assert time.monotonic() < deadline, "Camera subscribers did not connect"
            rclpy.spin_once(recorder, timeout_sec=0.01)
        bridge = CvBridge()
        for i in range(3):
            for pub, color in zip(publishers, colors):
                msg = bridge.cv2_to_imgmsg(np.full((height, width, 3), color, np.uint8), "bgr8")
                msg.header.stamp.sec = 12
                msg.header.stamp.nanosec = i * int(1_000_000_000 / fps)
                pub.publish(msg)
            while any(video.frames < i + 1 for video in recorder.videos.values()):
                assert time.monotonic() < deadline, "Camera frames did not arrive"
                rclpy.spin_once(recorder, timeout_sec=0.01)
    finally:
        recorder.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
    for name, color in zip(names, colors):
        capture = cv2.VideoCapture(str(tmp_path / f"{name}.avi"))
        try:
            assert capture.get(cv2.CAP_PROP_FPS) == fps
            for _ in range(3):
                ok, frame = capture.read()
                assert ok
                assert frame.shape[:2] == (height, width)
                assert np.allclose(frame[80, 80], color, atol=5)
            assert not capture.read()[0]
        finally:
            capture.release()
        timestamps = [json.loads(row) for row in (
            tmp_path / f"{name}.timestamps.jsonl"
        ).read_text().splitlines()]
        assert [row["stamp_ns"] for row in timestamps] == [
            12_000_000_000 + i * int(1_000_000_000 / fps) for i in range(3)
        ]


@pytest.mark.parametrize('overhead', [False, True])
def test_recording_waits_for_armed_takeoff_and_uses_each_vehicles_ground_level(tmp_path, overhead):
    vehicles = ['gate_alpha', 'gate_bravo']
    names = ['overhead'] if overhead else vehicles
    rclpy.init()
    recorder = CameraRecorder(names, tmp_path, 8, overhead=overhead, vehicle_names=vehicles)
    frame = CvBridge().cv2_to_imgmsg(np.zeros((32, 32, 3), np.uint8), 'bgr8')

    def images():
        for name in names:
            recorder.image(name, frame)

    def pose(name, z):
        msg = PoseStamped()
        msg.pose.position.z = z
        recorder.pose(name, msg)

    try:
        images()
        for name in vehicles:
            recorder.state(name, State(connected=True, armed=False))
            pose(name, 10.0)
        if overhead:
            assert not recorder.image_subscriptions
        images()
        for name in vehicles:
            recorder.state(name, State(connected=True, armed=True))
        if overhead:
            assert len(recorder.image_subscriptions) == 1
        images()  # Armed on the ground is still idle.
        assert all(v.frames == 0 for v in recorder.videos.values())
        assert not list(tmp_path.glob('*.avi'))
        pose(vehicles[1], 10.3)  # Only bravo is flying.
        images()
        if overhead:
            assert recorder.videos['overhead'].frames == 1
        else:
            assert recorder.videos[vehicles[0]].frames == 0
            assert recorder.videos[vehicles[1]].frames == 1
        pose(vehicles[0], 10.3)
        images()
        assert all(v.frames > 0 for v in recorder.videos.values())
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('count', [3, 6])
def test_ros_combined_recording_uses_one_takeoff_gate_and_one_video(tmp_path, count):
    names = [f'combined_uav{i}' for i in range(count)]
    colors = [(20 + i * 30, 210, 90) for i in range(count)]
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    source = rclpy.create_node('combined_camera_test_source')
    recorder = CameraRecorder(names, tmp_path, 20, combined=True)
    bridge = CvBridge()
    background = np.full((1080, 1920, 3), 70, np.uint8)
    try:
        assert set(recorder.image_subscriptions) == set(names)
        assert set(recorder.videos) == {'combined'}
        msg = bridge.cv2_to_imgmsg(background, 'bgr8')
        msg.header.stamp.sec = 12
        recorder.image('overhead', msg)
        assert not list(tmp_path.iterdir())
        # Other drones' FOVs share the first drone's gate, even before they lift.
        recorder.state(names[0], State(connected=True, armed=False))
        pose = PoseStamped()
        pose.pose.position.z = 10.0
        recorder.pose(names[0], pose)
        recorder.state(names[0], State(connected=True, armed=True))
        pose.pose.position.z = 10.3
        recorder.pose(names[0], pose)
        topics = ['/overhead/camera/image_raw',
                  *(f'/{name}/camera/image_raw' for name in names)]
        publishers = [source.create_publisher(Image, topic, qos_profile_sensor_data)
                      for topic in topics]
        deadline = time.monotonic() + 10
        while any(pub.get_subscription_count() == 0 for pub in publishers):
            assert time.monotonic() < deadline, 'Combined camera subscribers did not connect'
            rclpy.spin_once(recorder, timeout_sec=0.01)
        for i in range(2):
            frames = [background, *(np.full((160, 160, 3), color, np.uint8)
                                     for color in colors)]
            for pub, frame in zip(publishers, frames):
                msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp.sec = 12
                msg.header.stamp.nanosec = i * 100_000_000
                pub.publish(msg)
            while recorder.videos['combined'].frames < 1 + i * 2:
                assert time.monotonic() < deadline, 'Combined camera frames did not arrive'
                rclpy.spin_once(recorder, timeout_sec=0.01)
    finally:
        recorder.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
    path = tmp_path / 'combined.avi'
    capture = cv2.VideoCapture(str(path))
    try:
        assert capture.get(cv2.CAP_PROP_FPS) == 20
        assert capture.get(cv2.CAP_PROP_FRAME_COUNT) == 3
        ok, frame = capture.read()
        assert ok and frame.shape == (1080, 1920, 3)
        assert np.allclose(frame[540, 20], (70, 70, 70), atol=6)
        previous_y = -1
        for color in colors:
            matches = np.all(np.abs(frame.astype(float) - color) < 8, axis=2)
            # Ignore isolated JPEG edge pixels that resemble another panel.
            ys = np.where(matches.sum(axis=1) > 50)[0]
            assert len(ys) > 50
            assert not matches[:, :1440].any() and ys.min() > previous_y
            previous_y = ys.max()
    finally:
        capture.release()
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        'combined.avi', 'combined.timestamps.jsonl',
    ]


def test_combined_flush_timer_runs_while_ros_simulation_clock_is_paused(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    recorder = CameraRecorder(['timer_uav'], tmp_path, 20, combined=True)
    try:
        recorder.state('timer_uav', State(connected=True, armed=False))
        pose = PoseStamped()
        recorder.pose('timer_uav', pose)
        recorder.state('timer_uav', State(connected=True, armed=True))
        pose.pose.position.z = 0.3
        recorder.pose('timer_uav', pose)
        recorder.composite.max_wait = 0.01
        msg = CvBridge().cv2_to_imgmsg(np.zeros((360, 640, 3), np.uint8), 'bgr8')
        msg.header.stamp.sec = 12
        recorder.image('overhead', msg)
        assert recorder.videos['combined'].frames == 0
        deadline = time.monotonic() + 2
        while recorder.videos['combined'].frames == 0:
            assert time.monotonic() < deadline, 'Wall timer stalled with paused ROS clock'
            rclpy.spin_once(recorder, timeout_sec=0.1)
        assert recorder.get_clock().now().nanoseconds == 0
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


def test_map_overlay_uses_only_aligned_past_poses_and_ignores_old_maps(tmp_path, monkeypatch):
    from nav_msgs.msg import OccupancyGrid

    panels = []

    def render(cells, resolution, origin, positions, names, stamp):
        panels.append((cells.copy(), origin, positions, stamp))
        return np.zeros((100, 100, 3), np.uint8)

    monkeypatch.setattr('solo_mpc.camera_recorder.render_occupancy', render)
    rclpy.init()
    recorder = CameraRecorder(['alpha'], tmp_path, 20, combined=True)
    try:
        for frame_id, at, x in [('world', 10, -2.), ('world', 12, 8.), ('map', 13, 99.)]:
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp.sec = at
            msg.pose.position.x = x
            recorder.map_pose('alpha', msg)
        grid = OccupancyGrid()
        grid.header.frame_id = 'world'
        grid.header.stamp.sec = 11
        grid.info.width, grid.info.height = 3, 2
        grid.info.resolution = 0.5
        grid.info.origin.position.x = -10.
        grid.data = [-1, 0, 100, 0, -1, 0]
        recorder.occupancy(grid)
        assert panels[0][1:] == ((-10., 0.), {'alpha': (-2., 0.)}, 11_000_000_000)
        np.testing.assert_array_equal(panels[0][0], [[-1, 0, 100], [0, -1, 0]])
        grid.header.stamp.sec = 10
        recorder.occupancy(grid)
        assert len(panels) == 1
        grid.header.stamp.sec = 20
        recorder.occupancy(grid)
        assert panels[-1][2] == {}  # Old poses do not pin a UAV marker forever.
        grid.header.stamp.sec = 21
        grid.data = []
        recorder.occupancy(grid)
        assert len(panels) == 2
        assert not list(tmp_path.iterdir())  # Maps alone never create a video.
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
