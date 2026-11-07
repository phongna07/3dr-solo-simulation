"""Optional separate or synchronized combined recording of scene/FOV feeds."""

import argparse
from collections import deque
import json
from pathlib import Path
import signal
import time

import numpy as np
import yaml
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from solo_mpc_interfaces.msg import FlightState
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from solo_bringup.configuration import load_config
from solo_bringup.startup import ComponentProgress, write_status

from .core.video.encoder import FrameVideo
from .core.video.composite import CompositeVideo
from .core.video.occupancy import OCCUPANCY_TOPIC, render_occupancy
from .core.video.style import OVERHEAD_TOPIC


class CameraRecorder(Node):
    def __init__(self, names, output_dir, fps, *, overhead=False, combined=False,
                 vehicle_names=None):
        super().__init__("mpc_combined_recorder" if combined else
                         "mpc_overhead_recorder" if overhead else "mpc_camera_recorder")
        self.bridge = CvBridge()
        self.output_dir = Path(output_dir)
        self.recording_closed = False
        self.recording_status('waiting')
        self.vehicle_names = tuple(vehicle_names or names)
        self.overhead = overhead
        self.composite = None
        if combined:
            # The launch snapshots the resolved run inputs before any node starts.
            config = self.output_dir / 'config'
            self.composite = CompositeVideo(
                Path(output_dir) / 'recorded-mission.mp4', fps, self.vehicle_names,
                occupancy=True, telemetry=True,
                timestamps_path=Path(output_dir) / 'telemetry/recorded-mission.timestamps.jsonl',
                logger=self.get_logger(),
                map_config=json.loads((config / 'mpc-config.json').read_text()),
                algorithm=yaml.safe_load((config / 'controller.yaml').read_text())['algorithm'],
            )
        self.videos = ({'combined': self.composite.video} if combined else {
            name: FrameVideo(Path(output_dir) / f"{name}.avi", fps, logger=self.get_logger())
            for name in names
        })
        self.flush_timer = (self.create_timer(
            0.1, self.flush_composite, clock=Clock(clock_type=ClockType.STEADY_TIME),
        ) if combined else None)
        self.states = {}
        self.window = None
        self.subscriptions_keep = [self.create_subscription(
            String, '/mission/tracking_window', self.tracking_window,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        self.image_subscriptions = {}
        self.map_positions = {name: deque(maxlen=256) for name in self.vehicle_names}
        if combined:
            self.subscriptions_keep.append(self.create_subscription(
                String, '/mpc/status', self.controller_status, 10))
            self.subscriptions_keep.append(self.create_subscription(
                OccupancyGrid, OCCUPANCY_TOPIC, self.occupancy, qos_profile_sensor_data))
            for name in self.vehicle_names:
                self.subscriptions_keep.append(self.create_subscription(
                    PoseStamped, f'/{name}/mpc/pose',
                    lambda msg, n=name: self.map_pose(n, msg), qos_profile_sensor_data))
                for typ, suffix, callback in (
                    (TwistStamped, 'velocity', self.velocity),
                    (JointTrajectory, 'trajectory', self.reference),
                ):
                    self.subscriptions_keep.append(self.create_subscription(
                        typ, f'/{name}/mpc/{suffix}',
                        lambda msg, n=name, cb=callback: cb(n, msg), qos_profile_sensor_data))
        if not overhead:
            for name in names:
                self.subscribe_images(name)
        for name in self.vehicle_names:
            self.subscriptions_keep.append(self.create_subscription(
                FlightState, f'/{name}/flight/state',
                lambda msg, n=name: self.state(n, msg), qos_profile_sensor_data,
            ))
        self.get_logger().info(f"Waiting for mission tracking before recording in {output_dir}")

    def tracking_window(self, msg):
        try:
            window = json.loads(msg.data)
            begin, end = int(window['begin_ns']), window['end_ns']
            self.window = (begin, None if end is None else int(end))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warning('Ignoring invalid mission tracking window')
            return
        self.get_logger().info(f"Mission tracking window: {self.window}")

    def state(self, name, msg):
        self.states[name] = (msg, time.monotonic())
        if (self.overhead or self.composite) and msg.connected and msg.armed:
            # Start transport on arming, ahead of the takeoff recording gate,
            # while avoiding Full HD rendering before flight startup.
            self.subscribe_images('overhead')

    def subscribe_images(self, name):
        if name not in self.image_subscriptions:
            is_overhead = self.overhead or (self.composite and name == 'overhead')
            subscription = self.create_subscription(
                Image, OVERHEAD_TOPIC if is_overhead else f"/{name}/camera/image_raw",
                # Request retransmission for fragmented Full HD recording
                # images. Keep the small flight/perception feeds best-effort.
                lambda msg, n=name: self.image(n, msg),
                QoSProfile(depth=20) if is_overhead else qos_profile_sensor_data,
            )
            self.image_subscriptions[name] = subscription
            self.subscriptions_keep.append(subscription)

    def map_pose(self, name, msg):
        if msg.header.frame_id != 'world':
            return
        p = msg.pose.position
        if not np.isfinite([p.x, p.y]).all():
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        history = self.map_positions[name]
        if not history or stamp > history[-1][0]:
            history.append((stamp, (p.x, p.y)))
        q = msg.pose.orientation
        self.composite.write_state(name, 'pose', (p.x, p.y, p.z, q.x, q.y, q.z, q.w), stamp)

    def velocity(self, name, msg):
        if msg.header.frame_id != 'world':
            return
        v = msg.twist.linear
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.composite.write_state(name, 'velocity', (v.x, v.y, v.z), stamp)

    def reference(self, name, msg):
        if msg.header.frame_id != 'world' or not msg.points or len(msg.points[0].accelerations) != 2:
            return
        acceleration = msg.points[0].accelerations
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.composite.write_state(name, 'command', (*acceleration, 0.0), stamp)

    def controller_status(self, msg):
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(record, dict):
            self.composite.write_status(record)

    def occupancy(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if (msg.header.frame_id != 'world'
                or stamp <= self.composite.latest.get('occupancy', -1)):
            return
        info = msg.info
        if (info.width == 0 or info.height == 0 or not np.isfinite(info.resolution)
                or info.resolution <= 0 or len(msg.data) != info.width * info.height):
            self.get_logger().warning('Ignoring invalid occupancy grid metadata')
            return
        cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        positions = {}
        for name, history in self.map_positions.items():
            for at, position in reversed(history):
                if at <= stamp:
                    if stamp - at <= 1_000_000_000:
                        positions[name] = position
                    break
        panel = render_occupancy(
            cells, info.resolution, (info.origin.position.x, info.origin.position.y),
            positions, self.vehicle_names, stamp)
        self.composite.write('occupancy', panel, stamp)

    def in_window(self, stamp_ns):
        return bool(self.window and self.window[0] <= stamp_ns
                    and (self.window[1] is None or stamp_ns <= self.window[1]))

    def image(self, name, msg):
        if self.recording_closed:
            return
        key = 'combined' if self.composite else name
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        # Record only mission tracking, omitting takeoff and landing. The
        # overhead feed sets the composite timeline; FOVs may buffer early
        # so their panels hold current images at the first output frame.
        if not self.composite or name == 'overhead':
            if not self.in_window(stamp_ns):
                return
        elif self.window and self.window[1] is not None and stamp_ns > self.window[1]:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"{name}: cannot convert camera frame: {exc}")
            return
        video = self.videos[key]
        before = video.frames
        if self.composite:
            self.composite.write(name, frame, stamp_ns)
        else:
            video.write(frame, stamp_ns)
        if before == 0 and video.frames:
            self.get_logger().info(f"{key}: writing {video.path} at {video.fps:g} FPS")

    def flush_composite(self):
        if self.composite and not self.recording_closed:
            self.composite.flush()

    def recording_status(self, state, detail='', window=None):
        record = {'state': state, 'detail': detail, 'updated_wall_unix': time.time()}
        if window:
            record['tracking_window_ns'] = list(window)
        write_status(self.output_dir / 'status/recording.json', record)

    def finish_recording(self):
        if self.recording_closed:
            return
        self.recording_closed = True
        self.recording_status('finalizing')
        try:
            if self.composite:
                self.composite.close()
            for name, video in self.videos.items():
                video.close()
                self.get_logger().info(f"{name}: saved {video.frames} camera frames")
            summaries = [v.summary for v in self.videos.values() if v.summary]
            state = ('partial' if any(s['state'] == 'partial' for s in summaries) else
                     'finalized' if any(v.frames for v in self.videos.values()) else 'empty')
            self.recording_status(state, json.dumps(summaries), window=self.window)
        except BaseException as exc:
            self.recording_status('failed', f'{type(exc).__name__}: {exc}')
            raise

    def destroy_node(self):
        self.finish_recording()
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
    # Finish the current frame and its timestamp before flushing on shutdown.
    # A KeyboardInterrupt between those writes can leave an extra video frame.
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    progress = None
    shutdown_requested = False
    previous_signals = {}

    def shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    if args.run_dir and args.startup_component:
        progress = ComponentProgress(args.run_dir, args.startup_component)
    try:
        if progress:
            progress.update('initializing', 'Creating camera and flight telemetry subscriptions')
        node = CameraRecorder(names, args.output_dir, args.fps, overhead=args.overhead,
                              combined=args.combined, vehicle_names=vehicles)
        if progress:
            progress.update('initialized', 'Waiting for mission tracking')
        previous_signals = {
            sig: signal.signal(sig, shutdown) for sig in (signal.SIGINT, signal.SIGTERM)
        }
        while rclpy.ok() and not shutdown_requested:
            if node.recording_closed:
                # Finalization happens before stack shutdown. Do not dispatch
                # queued image/map/state callbacks against the closed video.
                time.sleep(0.1)
                continue
            rclpy.spin_once(node, timeout_sec=0.1)
            if (Path(args.output_dir) / 'status/finalize-recording.request').exists():
                node.finish_recording()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as exc:
        if node is not None:
            node.recording_status('failed', f'{type(exc).__name__}: {exc}')
        if progress:
            progress.update('failed', f'{type(exc).__name__}: {exc}')
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except BaseException as exc:
            if progress:
                progress.update('failed', f'Video finalization: {type(exc).__name__}: {exc}')
            raise
        finally:
            for sig, handler in previous_signals.items():
                signal.signal(sig, handler)
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
