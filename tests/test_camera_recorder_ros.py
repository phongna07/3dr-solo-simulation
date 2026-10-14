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

from solo_mpc.camera_recorder import CameraRecorder


def test_ros_camera_feeds_are_recorded_separately_and_closed_on_shutdown(tmp_path):
    names = ["camera_test_alpha", "camera_test_bravo", "camera_test_charlie"]
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 255)]
    rclpy.init()
    source = rclpy.create_node("camera_test_source")
    recorder = CameraRecorder(names, tmp_path, 8.0)
    try:
        assert {s.topic_name for s in recorder.subscriptions_keep} == {
            f"/{name}/camera/image_raw" for name in names
        }
        publishers = [source.create_publisher(
            Image, f"/{name}/camera/image_raw", qos_profile_sensor_data,
        ) for name in names]
        deadline = time.monotonic() + 10
        while any(pub.get_subscription_count() == 0 for pub in publishers):
            assert time.monotonic() < deadline, "Camera subscribers did not connect"
            rclpy.spin_once(recorder, timeout_sec=0.01)
        bridge = CvBridge()
        for i in range(3):
            for pub, color in zip(publishers, colors):
                msg = bridge.cv2_to_imgmsg(np.full((160, 160, 3), color, np.uint8), "bgr8")
                msg.header.stamp.sec = 12
                msg.header.stamp.nanosec = i * 125_000_000
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
            for _ in range(3):
                ok, frame = capture.read()
                assert ok
                assert np.allclose(frame[80, 80], color, atol=5)
            assert not capture.read()[0]
        finally:
            capture.release()
        timestamps = [json.loads(row) for row in (
            tmp_path / f"{name}.timestamps.jsonl"
        ).read_text().splitlines()]
        assert [row["stamp_ns"] for row in timestamps] == [
            12_000_000_000, 12_125_000_000, 12_250_000_000,
        ]
