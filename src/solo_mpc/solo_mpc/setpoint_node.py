"""Independent source-stamped trajectory sampling, expiry, and proximity guard."""

import json
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from solo_mpc_interfaces.msg import FlightState
from std_msgs.msg import String
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

    def trajectory(self, name, msg):
        if msg.header.frame_id == 'world' and msg.joint_names == ['x', 'y']:
            previous = self.trajectories.get(name)
            if previous is None or seconds(msg.header.stamp) > seconds(previous[0].header.stamp):
                self.trajectories[name] = msg, time.monotonic()

    def pose(self, name, msg):
        if msg.header.frame_id == 'world':
            self.poses[name] = msg, time.monotonic()

    def state(self, name, msg):
        self.states[name] = msg, time.monotonic()

    def enable(self, request, response):
        self.enabled = request.data
        self.trajectories.clear()
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
                    velocity[:2] = (feedforward
                                    + settings['position_kp'] * (target - position[:2])
                                    + settings['acceleration_feedforward_time'] * acceleration)
                    speed = np.linalg.norm(velocity[:2])
                    velocity[:2] *= min(1.0, cfg['VMAX'] / max(speed, 1e-9))
                    for other, (other_pose, received) in self.poses.items():
                        if other == name or wall - received > settings['pose_timeout_wall']:
                            continue
                        delta = pose_arrays(other_pose)[0][:2] - position[:2]
                        distance = np.linalg.norm(delta)
                        if (distance < cfg['UAV_SAFE_DISTANCE'] + 0.25
                                and np.dot(velocity[:2], delta) > 0):
                            reason = 'proximity_guard'
                            velocity[:2] = 0.0
                            break
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
        self.status.publish(String(data=json.dumps(dict(stamp=sim, vehicles=statuses))))


def main():
    run_node(Setpoints, 'Sample measured-state planning references independently of solvers')
