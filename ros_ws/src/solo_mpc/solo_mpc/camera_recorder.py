"""Optional recording of each UAV's rendered downward-facing camera feed."""

import argparse
from pathlib import Path

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from solo_bringup.configuration import load_config

from .core.video import FrameVideo


class CameraRecorder(Node):
    def __init__(self, names, output_dir, fps):
        super().__init__("mpc_camera_recorder")
        self.bridge = CvBridge()
        self.videos = {
            name: FrameVideo(Path(output_dir) / f"{name}.avi", fps) for name in names
        }
        self.subscriptions_keep = [
            self.create_subscription(
                Image, f"/{name}/camera/image_raw",
                lambda msg, n=name: self.image(n, msg), qos_profile_sensor_data,
            )
            for name in names
        ]
        self.get_logger().info(f"Recording UAV camera videos in {output_dir}")

    def image(self, name, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"{name}: cannot convert camera frame: {exc}")
            return
        stamp = msg.header.stamp
        video = self.videos[name]
        video.write(frame, stamp.sec * 1_000_000_000 + stamp.nanosec)
        if video.frames == 1:
            self.get_logger().info(f"{name}: writing {video.path} at {video.fps:g} FPS")

    def destroy_node(self):
        for name, video in self.videos.items():
            video.close()
            self.get_logger().info(f"{name}: saved {video.frames} camera frames")
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fleet", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--fps", type=float, required=True)
    args, ros = parser.parse_known_args()
    names = [uav["name"] for uav in load_config(args.fleet)["uavs"]]
    rclpy.init(args=ros)
    node = None
    try:
        node = CameraRecorder(names, args.output_dir, args.fps)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
