"""One vehicle's exclusive, measured-state native Gazebo flight controller."""

import math
import json
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from solo_bringup.startup import ComponentProgress
from solo_mpc_interfaces.msg import FlightState
from solo_mpc_interfaces.srv import FlightCommand
from std_msgs.msg import Bool
import yaml

from .ros_utils import arguments, seconds


class NativeFlight(Node):
    def __init__(self, args):
        if args.vehicle is None:
            raise ValueError('Flight requires --vehicle; run one process per UAV')
        super().__init__(f'{args.vehicle}_flight', parameter_overrides=[
            Parameter('use_sim_time', value=True),
        ])
        self.vehicle = args.vehicle
        self.settings = args.settings_data
        fleet = yaml.safe_load(Path(args.fleet).read_text())
        vehicle = next(item for item in fleet['uavs'] if item['name'] == self.vehicle)
        self.ground_z = float(vehicle['spawn'][2])
        self.ground_limit = self.ground_z + 0.15
        self.mode = 'IDLE'
        self.owner = ''
        self.armed = False
        self.enabled = False
        self.landed = False
        self.altitude = float(self.settings['flight_altitude'])
        self.anchor = np.zeros(2)
        self.position = None
        self.velocity = None
        self.angular_velocity = None
        self.rotation = None
        self.pose_stamp = None
        self.pose_wall = None
        self.motor_stamp = None
        self.motor_wall = None
        self.motor_peak = None
        self.motor_speeds = []
        self.telemetry = None
        if args.run_dir:
            path = Path(args.run_dir, 'telemetry', f'flight-{self.vehicle}.jsonl')
            self.telemetry = path.open('a', buffering=1)
        self.ground_since = None
        self.command = None
        self.command_stamp = None
        self.command_wall = None
        self.track_started = None
        self.arm_started = None
        self.takeoff_started = None
        self.takeoff_timeout_sim = None
        self.last_sim = None
        self.fault = ''
        self.safety_active = False
        self.disarming = False
        self.shutdown_requested = False
        self.shutdown_wall = None
        self.shutdown_sim = None
        self.shutdown_last_sim = None
        self.shutdown_clock_progress_wall = None
        self.conflict = False
        self.command_topic = f'/{self.vehicle}/cmd_vel'
        self.enable_topic = f'/{self.vehicle}/enable'
        self.command_publisher = self.create_publisher(Twist, self.command_topic, 1)
        self.enable_publisher = self.create_publisher(Bool, self.enable_topic, 1)
        self.state_publisher = self.create_publisher(
            FlightState, f'/{self.vehicle}/flight/state', 10)
        self.subscriptions_keepalive = [
            self.create_subscription(Odometry, f'/{self.vehicle}/ground_truth/odometry',
                                     self.odometry, qos_profile_sensor_data),
            self.create_subscription(Actuators, f'/{self.vehicle}/motor_speed',
                                     self.motors, qos_profile_sensor_data),
            self.create_subscription(TwistStamped, f'/{self.vehicle}/flight/velocity',
                                     self.reference, qos_profile_sensor_data),
            self.create_subscription(Bool, f'/{self.vehicle}/safety_active',
                                     self.safety, qos_profile_sensor_data),
        ]
        self.command_service = self.create_service(
            FlightCommand, f'/{self.vehicle}/flight/command', self.flight_command)
        self.timer = self.create_timer(
            1.0 / self.settings['publish_rate'], self.tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh(self, stamp, wall, timeout_sim, timeout_wall, future_tolerance=0.0):
        return (stamp is not None and wall is not None
                and -future_tolerance <= self.sim_time() - stamp <= timeout_sim
                and 0 <= time.monotonic() - wall <= timeout_wall)

    def pose_fresh(self):
        return self.fresh(self.pose_stamp, self.pose_wall,
                          self.settings['state_timeout_sim'], self.settings['pose_timeout_wall'], 0.05)

    def motors_fresh(self):
        return self.fresh(self.motor_stamp, self.motor_wall,
                          self.settings.get('flight_timeout_sim', 2.5),
                          self.settings.get('flight_timeout_wall', 60.0), 0.05)

    def connected(self):
        return (not self.conflict and self.pose_fresh() and self.motors_fresh()
                and self.command_publisher.get_subscription_count() > 0
                and self.enable_publisher.get_subscription_count() > 0)

    def odometry(self, msg):
        stamp = seconds(msg.header.stamp)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        values = [position.x, position.y, position.z, orientation.x, orientation.y,
                  orientation.z, orientation.w, linear.x, linear.y, linear.z,
                  angular.x, angular.y, angular.z]
        if msg.header.frame_id != 'world' or not all(math.isfinite(value) for value in values):
            return
        if (msg.header.frame_id != 'world' or stamp > self.sim_time() + 0.05
                or (self.pose_stamp is not None and stamp <= self.pose_stamp)):
            return
        quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        norm = np.linalg.norm(quaternion)
        if not 0.9 <= norm <= 1.1:
            return
        quaternion /= norm
        quat_x, quat_y, quat_z, quat_w = quaternion
        self.rotation = np.array([
            [1 - 2 * (quat_y**2 + quat_z**2), 2 * (quat_x * quat_y - quat_z * quat_w),
             2 * (quat_x * quat_z + quat_y * quat_w)],
            [2 * (quat_x * quat_y + quat_z * quat_w), 1 - 2 * (quat_x**2 + quat_z**2),
             2 * (quat_y * quat_z - quat_x * quat_w)],
            [2 * (quat_x * quat_z - quat_y * quat_w), 2 * (quat_y * quat_z + quat_x * quat_w),
             1 - 2 * (quat_x**2 + quat_y**2)],
        ])
        previous = self.pose_stamp
        self.position = np.array([position.x, position.y, position.z])
        self.velocity = self.rotation @ np.array([linear.x, linear.y, linear.z])
        self.angular_velocity = np.array([angular.x, angular.y, angular.z])
        self.pose_stamp, self.pose_wall = stamp, time.monotonic()
        grounded = (self.ground_z - 0.15 <= self.position[2]
                    <= min(self.ground_limit, self.ground_z + 0.1)
                    and np.linalg.norm(self.velocity) <= 0.12
                    and np.linalg.norm(self.angular_velocity) <= 0.2
                    and self.rotation[2, 2] >= math.cos(math.radians(15)))
        if (not grounded or previous is None
                or stamp - previous > self.settings['state_timeout_sim']):
            self.ground_since = None
        elif self.ground_since is None:
            self.ground_since = previous
        self.landed = self.ground_since is not None and stamp - self.ground_since >= 1.0
        if self.mode == 'IDLE' and not self.armed and self.landed:
            self.ground_z = min(self.ground_z, float(self.position[2]))

    def motors(self, msg):
        stamp = seconds(msg.header.stamp)
        speeds = list(msg.velocity)
        if (len(speeds) != 4 or not all(math.isfinite(value) for value in speeds)
                or stamp > self.sim_time() + 0.05
                or (self.motor_stamp is not None and stamp <= self.motor_stamp)):
            return
        self.motor_stamp, self.motor_wall = stamp, time.monotonic()
        self.motor_peak = max(abs(value) for value in speeds)
        self.motor_speeds = speeds
        if self.motor_peak > 1.0:
            self.armed = True

    def safety(self, msg):
        self.safety_active = msg.data
        if self.safety_active:
            self.begin_landing('Gazebo safety watchdog latched')

    def reference(self, msg):
        stamp = seconds(msg.header.stamp)
        linear, angular = msg.twist.linear, msg.twist.angular
        values = [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
        if (self.mode != 'TRACK' or msg.header.frame_id != 'world'
                or not all(math.isfinite(value) for value in values)
                or not 0 <= self.sim_time() - stamp <= self.settings['command_timeout_sim']
                or (self.command_stamp is not None and stamp < self.command_stamp)):
            return
        if stamp != self.command_stamp:
            self.command_wall = time.monotonic()
        self.command, self.command_stamp = msg, stamp

    def begin_landing(self, reason=''):
        if self.mode != 'LAND':
            self.anchor = self.position[:2].copy() if self.position is not None else np.zeros(2)
        self.mode = 'LAND'
        self.command = None
        if reason and not self.fault:
            self.fault = reason
            self.get_logger().error(f'{self.vehicle}: {reason}; LAND')

    def flight_command(self, request, response):
        command = request.command.lower()
        if not request.owner or len(request.owner) > 128:
            response.message = 'A nonempty owner token (at most 128 characters) is required'
        elif command == 'acquire':
            if self.shutdown_requested or self.safety_active:
                response.message = 'Flight backend is shutting down or Gazebo safety remains active'
            elif self.owner and self.owner != request.owner:
                response.message = 'Flight belongs to another owner'
            elif not self.connected() or self.mode != 'IDLE' or self.armed or not self.landed:
                response.message = 'Acquire requires fresh, connected, settled, disarmed IDLE flight'
            else:
                self.owner = request.owner
                self.disarming = False
                self.fault = ''
                response.success = True
        elif self.owner != request.owner:
            response.message = 'Flight belongs to another owner; command rejected'
        elif command == 'land':
            self.begin_landing()
            response.success = True
        elif self.shutdown_requested or self.fault:
            response.message = f'Flight backend unavailable: {self.fault or "shutting down"}'
        elif command == 'arm':
            if self.connected() and self.mode == 'IDLE' and self.landed:
                self.armed = self.enabled = True
                self.arm_started = (self.sim_time(), time.monotonic())
                response.success = True
            else:
                response.message = 'Arm requires connected, settled IDLE flight'
        elif command == 'takeoff':
            if not math.isfinite(request.altitude) or request.altitude <= self.ground_z + 0.3:
                response.message = 'Takeoff altitude must be finite and above the ground'
            elif self.mode != 'IDLE' or not self.enabled or not self.connected():
                response.message = 'Takeoff requires armed, connected IDLE flight'
            else:
                self.altitude = request.altitude
                self.anchor = self.position[:2].copy()
                self.mode = 'TAKEOFF'
                self.takeoff_started = (self.sim_time(), time.monotonic())
                self.takeoff_timeout_sim = max(60.0, 3.0 * abs(self.altitude - self.position[2])
                                              / self.settings['vertical_speed_limit'] + 30.0)
                self.landed = False
                self.ground_since = None
                response.success = True
        elif command == 'track':
            if (self.mode != 'TAKEOFF' or not self.enabled or not self.connected()
                    or abs(self.position[2] - self.altitude) > 0.5
                    or np.linalg.norm(self.velocity) > 0.3):
                response.message = 'Track requires connected, settled takeoff altitude'
            else:
                self.mode = 'TRACK'
                self.command = self.command_stamp = self.command_wall = None
                self.track_started = (self.sim_time(), time.monotonic())
                response.success = True
        elif command == 'release':
            if self.armed or not self.landed or self.mode != 'IDLE':
                response.message = 'Release requires settled, disarmed IDLE flight'
            else:
                self.owner = ''
                response.success = True
        else:
            response.message = f'Unknown flight command: {request.command}'
        if response.success:
            response.message = f'{self.vehicle}: {command} accepted'
        self.publish_state()
        return response

    def publish_state(self):
        state = FlightState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'
        state.connected = self.connected()
        state.armed, state.mode, state.owner = self.armed, self.mode, self.owner
        state.landed = self.landed and self.pose_fresh()
        self.state_publisher.publish(state)
        if self.telemetry:
            self.telemetry.write(json.dumps(dict(
                stamp=self.sim_time(), vehicle=self.vehicle, mode=self.mode, owner=self.owner,
                armed=self.armed, enabled=self.enabled, landed=state.landed,
                connected=state.connected, safety_active=self.safety_active, fault=self.fault,
                pose_source_stamp=self.pose_stamp, motor_source_stamp=self.motor_stamp,
                position=None if self.position is None else self.position.tolist(),
                velocity=None if self.velocity is None else self.velocity.tolist(),
                motor_speed_rad_s=self.motor_speeds,
            )) + '\n')

    def destroy_node(self):
        if self.telemetry:
            self.telemetry.close()
        return super().destroy_node()

    def tick(self):
        sim, wall = self.sim_time(), time.monotonic()
        if self.last_sim is not None and sim < self.last_sim:
            self.command = None
            self.pose_stamp = self.motor_stamp = None
            self.ground_since = None
            self.landed = False
            if self.armed:
                self.begin_landing('Simulation clock moved backwards')
        self.last_sim = sim
        self.conflict = (self.count_publishers(self.command_topic) > 1
                         or self.count_publishers(self.enable_topic) > 1)
        if self.conflict and self.armed:
            self.begin_landing('Another publisher is commanding native flight')
        if self.armed and (self.shutdown_requested or not self.connected()):
            self.begin_landing('Shutdown requested' if self.shutdown_requested
                               else 'Native flight telemetry or command bridge stale/unavailable')
        if self.armed and self.mode == 'IDLE' and not self.enabled:
            self.begin_landing('Unowned rotor motion detected')
        if self.armed and self.mode == 'IDLE' and self.arm_started is not None:
            if not self.fresh(*self.arm_started, self.settings['command_timeout_sim'],
                              self.settings['command_timeout_wall']):
                self.begin_landing('Armed flight did not receive takeoff command')
        if self.mode == 'TAKEOFF':
            stamp, received = self.takeoff_started
            if (sim - stamp > self.takeoff_timeout_sim
                    or wall - received > self.takeoff_timeout_sim
                    / self.settings.get('simulation_real_time_factor', 0.2)
                    + self.settings.get('landing_stall_timeout_wall', 60.0)):
                self.begin_landing('Takeoff/hold deadline exceeded without TRACK command')
        if self.mode == 'TRACK':
            if self.command is None:
                stamp, received = self.track_started
            else:
                stamp, received = self.command_stamp, self.command_wall
            if not self.fresh(stamp, received, self.settings['command_timeout_sim'],
                              self.settings['command_timeout_wall']):
                self.begin_landing('Source-stamped velocity command expired')
        if not self.pose_fresh():
            self.publish_state()
            return
        if self.mode == 'LAND' and self.landed:
            self.disarming = True
        if self.mode == 'LAND' and self.disarming:
            self.enabled = False
            if self.landed and self.motors_fresh() and self.motor_peak <= 1.0:
                self.armed = False
                self.owner = ''
                self.mode = 'IDLE'
        world_velocity = np.zeros(3)
        world_angular = np.zeros(3)
        if self.mode in ('TAKEOFF', 'TRACK', 'LAND') and self.armed:
            if self.mode == 'TRACK' and self.command is not None:
                linear, angular = self.command.twist.linear, self.command.twist.angular
                world_velocity[:] = [linear.x, linear.y, linear.z]
                world_angular[:] = [angular.x, angular.y, angular.z]
            else:
                world_velocity[:2] = self.settings['altitude_kp'] * (self.anchor - self.position[:2])
                if self.mode == 'LAND':
                    world_velocity[2] = -min(self.settings['vertical_speed_limit'], max(0.12,
                        self.settings['altitude_kp'] * (self.position[2] - self.ground_z)))
                else:
                    world_velocity[2] = self.settings['altitude_kp'] * (self.altitude - self.position[2])
            horizontal_speed = np.linalg.norm(world_velocity[:2])
            max_speed = float(self.settings.get('params', {}).get('VMAX', 1.2))
            if horizontal_speed > max_speed:
                world_velocity[:2] *= max_speed / horizontal_speed
            world_velocity[2] = np.clip(world_velocity[2], -self.settings['vertical_speed_limit'],
                                        self.settings['vertical_speed_limit'])
            world_angular = np.clip(world_angular, -1.0, 1.0)
        if self.mode == 'LAND' and self.armed and not self.disarming:
            self.enabled = True
        command = Twist()
        body_velocity = self.rotation.T @ world_velocity
        body_angular = self.rotation.T @ world_angular
        command.linear.x, command.linear.y, command.linear.z = map(float, body_velocity)
        command.angular.x, command.angular.y, command.angular.z = map(float, body_angular)
        self.command_publisher.publish(command)
        if self.enabled or self.landed or self.disarming:
            self.enable_publisher.publish(Bool(data=self.enabled))
        self.publish_state()

    def request_shutdown(self):
        self.shutdown_requested = True
        if self.shutdown_wall is None:
            self.shutdown_wall = self.shutdown_clock_progress_wall = time.monotonic()
            self.shutdown_sim = self.shutdown_last_sim = self.sim_time()
        if self.armed:
            self.begin_landing('Flight process shutdown requested')


def main(args=None):
    options, ros = arguments(__doc__)
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    progress = (ComponentProgress(options.run_dir, options.startup_component)
                if options.run_dir and options.startup_component else None)
    previous_signals = {}
    node = None
    try:
        if progress:
            progress.update('initializing', f'Constructing native flight for {options.vehicle}')
        node = NativeFlight(options)
        if progress:
            progress.update('initialized', 'Native flight services ready; waiting for measured state')
        previous_signals = {
            sig: signal.signal(sig, lambda signum, frame: node.request_shutdown())
            for sig in (signal.SIGINT, signal.SIGTERM)
        }
        while rclpy.ok() and (not node.shutdown_requested or node.armed):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.shutdown_wall is not None and node.armed:
                sim, wall = node.sim_time(), time.monotonic()
                if sim > node.shutdown_last_sim:
                    node.shutdown_clock_progress_wall = wall
                node.shutdown_last_sim = sim
                stall_timeout = node.settings.get('landing_stall_timeout_wall', 60.0)
                if (sim - node.shutdown_sim >= node.settings.get('landing_timeout_sim', 60.0)
                        or wall - node.shutdown_clock_progress_wall >= stall_timeout
                        or node.pose_wall is None or wall - node.pose_wall >= stall_timeout):
                    raise RuntimeError('Native flight shutdown could not confirm landed/disarmed; '
                                       'Gazebo safety watchdog retains landing responsibility')
    except BaseException as error:
        if progress:
            progress.update('failed', str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
