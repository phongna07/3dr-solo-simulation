"""Optional separate or synchronized combined recording of scene/FOV feeds."""

import argparse
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from solo_bringup.configuration import load_config
from solo_bringup.startup import ComponentProgress

from .core.video import FrameVideo
from .core.composite_video import CompositeVideo
from .scene_view import OVERHEAD_TOPIC


class CameraRecorder(Node):
    def __init__(self, names, output_dir, fps, *, overhead=False, combined=False,
                 vehicle_names=None):
        super().__init__("mpc_combined_recorder" if combined else
                         "mpc_overhead_recorder" if overhead else "mpc_camera_recorder")
        self.bridge = CvBridge()
        self.vehicle_names = tuple(vehicle_names or names)
        self.overhead = overhead
        self.composite = (CompositeVideo(Path(output_dir) / 'combined.avi', fps,
                                         self.vehicle_names) if combined else None)
        self.videos = ({'combined': self.composite.video} if combined else {
            name: FrameVideo(Path(output_dir) / f"{name}.avi", fps) for name in names
        })
        self.flush_timer = (self.create_timer(
            0.1, self.flush_composite, clock=Clock(clock_type=ClockType.STEADY_TIME),
        ) if combined else None)
        self.states = {}
        self.poses = {}
        self.ground_altitudes = {}
        self.recording = set()
        self.subscriptions_keep = []
        self.image_subscriptions = {}
        if not overhead:
            for name in names:
                self.subscribe_images(name)
        for name in self.vehicle_names:
            for typ, suffix, callback in (
                (State, 'state', self.state),
                (PoseStamped, 'local_position/pose', self.pose),
            ):
                self.subscriptions_keep.append(self.create_subscription(
                    typ, f'/{name}/mavros/{suffix}',
                    lambda msg, n=name, cb=callback: cb(n, msg), qos_profile_sensor_data,
                ))
        self.get_logger().info(f"Waiting for armed UAV takeoff before recording in {output_dir}")

    def state(self, name, msg):
        self.states[name] = (msg, time.monotonic())
        if (self.overhead or self.composite) and msg.connected and msg.armed:
            # Start transport on arming, ahead of the takeoff recording gate,
            # while avoiding Full HD rendering during EKF startup.
            self.subscribe_images('overhead')

    def subscribe_images(self, name):
        if name not in self.image_subscriptions:
            subscription = self.create_subscription(
                Image, OVERHEAD_TOPIC if self.overhead or (self.composite and name == 'overhead')
                else f"/{name}/camera/image_raw",
                lambda msg, n=name: self.image(n, msg), qos_profile_sensor_data,
            )
            self.image_subscriptions[name] = subscription
            self.subscriptions_keep.append(subscription)

    def pose(self, name, msg):
        z = msg.pose.position.z
        self.poses[name] = (z, time.monotonic())
        state = self.states.get(name)
        if state is None or not state[0].armed:
            self.ground_altitudes[name] = z

    def flying(self, name):
        state = self.states.get(name)
        pose = self.poses.get(name)
        now = time.monotonic()
        return bool(
            state and state[0].connected and state[0].armed
            and now - state[1] < 60.0 and pose and now - pose[1] < 3.0
            and name in self.ground_altitudes
            and pose[0] > self.ground_altitudes[name] + 0.15
        )

    def image(self, name, msg):
        key = 'combined' if self.composite else name
        if key not in self.recording:
            vehicles = self.vehicle_names if self.overhead or self.composite else (name,)
            if not any(self.flying(n) for n in vehicles):
                return
            self.recording.add(key)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"{name}: cannot convert camera frame: {exc}")
            return
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        video = self.videos[key]
        before = video.frames
        if self.composite:
            self.composite.write(name, frame, stamp_ns)
        else:
            video.write(frame, stamp_ns)
        if before == 0 and video.frames:
            self.get_logger().info(f"{key}: writing {video.path} at {video.fps:g} FPS")

    def flush_composite(self):
        if self.composite:
            self.composite.flush()

    def destroy_node(self):
        if self.composite:
            self.composite.close()
        for name, video in self.videos.items():
            video.close()
            self.get_logger().info(f"{name}: saved {video.frames} camera frames")
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fleet", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--overhead", action="store_true")
    mode.add_argument("--combined", action="store_true")
    parser.add_argument('--run-dir')
    parser.add_argument('--startup-component')
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--fps", type=float, required=True)
    args, ros = parser.parse_known_args()
    vehicles = [uav['name'] for uav in load_config(args.fleet)['uavs']]
    names = ['overhead'] if args.overhead else vehicles
    rclpy.init(args=ros)
    node = None
    progress = None
    if args.run_dir and args.startup_component:
        progress = ComponentProgress(args.run_dir, args.startup_component)
    try:
        if progress:
            progress.update('initializing', 'Creating camera and flight telemetry subscriptions')
        node = CameraRecorder(names, args.output_dir, args.fps, overhead=args.overhead,
                              combined=args.combined, vehicle_names=vehicles)
        if progress:
            progress.update('initialized', 'Waiting for armed UAV takeoff')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as exc:
        if progress:
            progress.update('failed', f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
