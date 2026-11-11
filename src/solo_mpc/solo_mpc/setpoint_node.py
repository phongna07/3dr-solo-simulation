"""Independent source-stamped trajectory sampling, expiry, solver lockstep and compute delay."""

import json
import time
from collections import Counter
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzNode
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from solo_mpc_interfaces.msg import FlightState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .core.planning.references import sample_trajectory
from .ros_utils import pose_arrays, qos_profile_sensor_data, run_node, seconds


class Setpoints(Node):
    def __init__(self, args):
        super().__init__('mpc_setpoints')
        self.args = args
        self.trajectories, self.poses, self.states = {}, {}, {}
        self.publishers_keep, self.subscriptions_keep = {}, []
        self.enabled = False
        self.create_service(SetBool, '/mpc/enable', self.enable)
        self.status = self.create_publisher(String, '/mpc/setpoint_status', 10)
        for name in args.names:
            self.publishers_keep[name] = self.create_publisher(
                TwistStamped, f'/{name}/flight/velocity', 10)
            for kind, suffix, callback in (
                (JointTrajectory, 'mpc/trajectory', self.trajectory),
                (PoseStamped, 'mpc/pose', self.pose),
                (FlightState, 'flight/state', self.state),
            ):
                self.subscriptions_keep.append(self.create_subscription(
                    kind, f'/{name}/{suffix}',
                    lambda msg, vehicle=name, handler=callback: handler(vehicle, msg),
                    qos_profile_sensor_data))
        self.create_timer(1.0 / args.settings_data['publish_rate'], self.tick)
        self.log = None
        if args.run_dir:
            Path(args.run_dir, 'telemetry').mkdir(parents=True, exist_ok=True)
            self.log = open(Path(args.run_dir, 'telemetry/setpoints.jsonl'), 'a', buffering=1)
        # This process, not the controller, owns the pause: a held world resumes at
        # the wall deadline even if the controller dies, so landing/expiry still run.
        self.lockstep = Counter()
        self.hold_deadline = None
        # Compute delay: solved plans wait in pending until the world, running on
        # the previous plans, reaches the release time (source stamp + fleet delay).
        self.compute_delay = args.settings_data['solver_compute_delay']
        self.pending, self.release, self.delivered_source = {}, None, -np.inf
        if args.settings_data['solver_lockstep']:
            self.gz = GzNode()
            self.create_subscription(Bool, '/mpc/lockstep_hold', self.hold, 10)
            if self.compute_delay:
                self.create_subscription(
                    Float64MultiArray, '/mpc/lockstep_release', self.release_world, 10)
            self.create_timer(0.01 if self.compute_delay else 0.05, self.hold_watchdog,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

    def control_world(self, pause, run_to=None):
        request = WorldControl()
        request.pause = pause
        if run_to is not None:
            # Gazebo pauses itself once simulation time reaches this value.
            nanoseconds = int(round(run_to * 1e9))
            request.run_to_sim_time.sec, request.run_to_sim_time.nsec = divmod(nanoseconds, 10**9)
        try:
            done, reply = self.gz.request(
                f'/world/{self.args.world}/control', request, WorldControl, Boolean, 500)
        except Exception as error:
            self.get_logger().error(f'World control failed: {error}')
            done, reply = False, None
        if not done or not reply.data:
            self.lockstep['service_failures'] += 1
            self.get_logger().error(f'World control request pause={pause} failed')

    def hold(self, msg):
        if msg.data:
            self.lockstep['holds'] += 1
            self.hold_deadline = time.monotonic() + self.args.settings_data['solver_lockstep_hold_wall']
        else:
            self.hold_deadline = None
        self.control_world(msg.data)

    def release_world(self, msg):
        source, at = msg.data
        sim = self.get_clock().now().nanoseconds * 1e-9
        self.lockstep['releases'] += 1
        # The world now runs, so the paused-hold deadline gives way to a
        # release deadline paced by the requested real-time factor.
        self.hold_deadline = None
        settings = self.args.settings_data
        self.release = source, at, time.monotonic() + settings['solver_lockstep_hold_wall'] + max(
            0.0, at - sim) / settings['simulation_real_time_factor']
        self.control_world(False, run_to=at)

    def deliver(self):
        source, _, _ = self.release
        self.release, self.delivered_source = None, source
        for name, (msg, _) in list(self.pending.items()):
            if seconds(msg.header.stamp) <= source:
                # The sampler receives the plan only now; its source stamp is unchanged.
                self.trajectories[name] = msg, time.monotonic()
                del self.pending[name]
        # Publish at the release instant, then resume the world Gazebo paused there.
        self.tick()
        self.control_world(False)

    def hold_watchdog(self):
        if self.release is not None:
            if self.get_clock().now().nanoseconds * 1e-9 >= self.release[1] - 5e-4:
                self.deliver()
            elif time.monotonic() > self.release[2]:
                self.lockstep['expired_releases'] += 1
                self.get_logger().error('Compute-delay release expired; delivering and resuming')
                self.deliver()
        if self.hold_deadline is not None and time.monotonic() > self.hold_deadline:
            self.hold_deadline = None
            self.lockstep['expired_holds'] += 1
            self.get_logger().error('Solver lockstep hold expired; resuming simulation')
            self.control_world(False)

    def trajectory(self, name, msg):
        if msg.header.frame_id == 'world' and msg.joint_names == ['x', 'y']:
            stamp = seconds(msg.header.stamp)
            store = (self.pending if self.compute_delay and stamp > self.delivered_source
                     else self.trajectories)
            previous = store.get(name)
            if previous is None or stamp > seconds(previous[0].header.stamp):
                store[name] = msg, time.monotonic()

    def pose(self, name, msg):
        if msg.header.frame_id == 'world':
            self.poses[name] = msg, time.monotonic()

    def state(self, name, msg):
        self.states[name] = msg, time.monotonic()

    def enable(self, request, response):
        self.enabled = request.data
        self.trajectories.clear()
        self.pending.clear()
        response.success = True
        response.message = 'enabled' if self.enabled else 'disabled'
        return response

    def tick(self):
        if not self.enabled:
            return
        now, wall = self.get_clock().now(), time.monotonic()
        sim = now.nanoseconds * 1e-9
        settings, cfg = self.args.settings_data, self.args.cfg
        statuses = {}
        for name in self.args.names:
            state = self.states.get(name)
            if not state or not state[0].connected or not state[0].armed or state[0].mode != 'TRACK':
                continue
            pose = self.poses.get(name)
            if pose is None:
                statuses[name] = 'missing_pose'
                continue
            position = pose_arrays(pose[0])[0]
            trajectory = self.trajectories.get(name)
            reason = 'tracking'
            if (wall - pose[1] > settings['pose_timeout_wall']
                    or not 0 <= sim - seconds(pose[0].header.stamp) <= settings['state_timeout_sim']
                    or wall - state[1] > settings['flight_timeout_wall']
                    or not 0 <= sim - seconds(state[0].header.stamp) <= settings['flight_timeout_sim']):
                reason = 'stale_pose_or_flight'
            elif (trajectory is None
                  or not 0 <= sim - seconds(trajectory[0].header.stamp) <= settings['command_timeout_sim']
                  or wall - trajectory[1] > settings['command_timeout_wall']):
                reason = 'expired_command'
            velocity = np.zeros(3)
            if reason == 'tracking':
                try:
                    target, feedforward, acceleration = sample_trajectory(
                        trajectory[0].points, sim - seconds(trajectory[0].header.stamp))
                    if (not np.isfinite(np.r_[target, feedforward, acceleration]).all()
                            or np.linalg.norm(feedforward) > cfg['VMAX'] * 1.05
                            or np.linalg.norm(acceleration) > cfg['UMAX'] * 1.05):
                        raise ValueError('invalid trajectory values or limits')
                    velocity[:2] = ((1.0 + settings['velocity_drag_feedforward']) * feedforward
                                    + settings['position_kp'] * (target - position[:2])
                                    + settings['acceleration_feedforward_time'] * acceleration)
                    speed = np.linalg.norm(velocity[:2])
                    velocity[:2] *= min(1.0, cfg['VMAX'] / max(speed, 1e-9))
                except (ValueError, IndexError, TypeError):
                    reason = 'invalid_command'
                    velocity[:] = 0.0
            velocity[2] = float(np.clip(settings['altitude_kp'] * (
                settings['flight_altitude'] - position[2]),
                -settings['vertical_speed_limit'], settings['vertical_speed_limit']))
            if reason == 'stale_pose_or_flight':
                statuses[name] = reason
                continue
            command = TwistStamped()
            command.header.stamp = now.to_msg()
            command.header.frame_id = 'world'
            command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = map(float, velocity)
            self.publishers_keep[name].publish(command)
            statuses[name] = reason
        text = json.dumps(dict(stamp=sim, vehicles=statuses, lockstep=self.lockstep))
        self.status.publish(String(data=text))
        if self.log:
            self.log.write(text + '\n')

    def destroy_node(self):
        if self.hold_deadline is not None or self.release is not None:
            self.control_world(False)
        if self.log:
            self.log.close()
        return super().destroy_node()


def main():
    run_node(Setpoints, 'Sample measured-state planning references independently of solvers')
