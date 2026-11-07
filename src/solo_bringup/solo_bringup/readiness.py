"""Monitor native flight, source-stamped ground truth and actual rotor telemetry."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from solo_mpc_interfaces.msg import FlightState
from solo_mpc_interfaces.srv import FlightCommand
from std_msgs.msg import Bool, String

from solo_bringup.configuration import load_config


def atomic_json(path, payload):
    path = Path(path)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


class Readiness(Node):
    def __init__(self, config, run_dir, max_pose_age, state_timeout=60.0):
        super().__init__('fleet_readiness')
        self.run_dir = Path(run_dir)
        (self.run_dir / 'status').mkdir(parents=True, exist_ok=True)
        self.max_pose_age = max_pose_age
        self.state_timeout = state_timeout
        self.started = time.monotonic()
        self.previous_ready = None
        self.previous_summary = None
        self.last_report = 0
        self.sim_time = None
        self.clock_at = None
        self.clock_samples = deque()
        self.first_ready_elapsed = None
        self.records = {}
        self.publisher = self.create_publisher(
            String, '/solo/readiness',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions_keepalive = [self.create_subscription(
            Clock, '/clock', self.clock, qos_profile_sensor_data)]
        for uav in config['uavs']:
            name = uav['name']
            self.records[name] = {
                'state': None, 'state_at': None, 'pose_at': None, 'pose_count': 0,
                'stamp': None, 'motor_stamp': None, 'motor_at': None, 'motor_speed': None,
                'safety_active': False,
                'client': self.create_client(FlightCommand, f'/{name}/flight/command'),
            }
            self.subscriptions_keepalive += [
                self.create_subscription(FlightState, f'/{name}/flight/state',
                                         lambda msg, n=name: self.state(msg, n), qos_profile_sensor_data),
                self.create_subscription(Odometry, f'/{name}/ground_truth/odometry',
                                         lambda msg, n=name: self.pose(msg, n), qos_profile_sensor_data),
                self.create_subscription(Actuators, f'/{name}/motor_speed',
                                         lambda msg, n=name: self.motors(msg, n), qos_profile_sensor_data),
                self.create_subscription(Bool, f'/{name}/safety_active',
                                         lambda msg, n=name: self.safety(msg, n), qos_profile_sensor_data),
            ]
        self.timer = self.create_timer(0.5, self.tick)

    def clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_time is not None and value < self.sim_time:
            self.clock_samples.clear()
            for record in self.records.values():
                record.update(state=None, state_at=None, pose_at=None, pose_count=0,
                              stamp=None, motor_stamp=None, motor_at=None, motor_speed=None)
        self.sim_time, self.clock_at = value, time.monotonic()

    def simulation_status(self, now):
        if self.sim_time is None:
            return {'time_s': None, 'real_time_factor': None, 'clock_age_wall_s': None}
        self.clock_samples.append((now, self.sim_time))
        while len(self.clock_samples) > 2 and now - self.clock_samples[1][0] >= 10:
            self.clock_samples.popleft()
        wall, sim = self.clock_samples[0]
        factor = (self.sim_time - sim) / (now - wall) if now - wall >= 2 else None
        return {'time_s': self.sim_time, 'real_time_factor': factor,
                'clock_age_wall_s': now - self.clock_at}

    def state(self, msg, name):
        self.records[name]['state'], self.records[name]['state_at'] = msg, time.monotonic()

    def pose(self, msg, name):
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        values = (position.x, position.y, position.z, orientation.x, orientation.y,
                  orientation.z, orientation.w, linear.x, linear.y, linear.z,
                  angular.x, angular.y, angular.z)
        if msg.header.frame_id != 'world' or not all(math.isfinite(value) for value in values):
            return
        norm = sum(value**2 for value in (orientation.x, orientation.y, orientation.z, orientation.w))
        if not 0.81 <= norm <= 1.21:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        record = self.records[name]
        if record['stamp'] is None or stamp > record['stamp']:
            record['pose_count'] += 1
            record['stamp'], record['pose_at'] = stamp, time.monotonic()

    def motors(self, msg, name):
        if len(msg.velocity) != 4 or not all(math.isfinite(value) for value in msg.velocity):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        record = self.records[name]
        if record['motor_stamp'] is None or stamp > record['motor_stamp']:
            record['motor_stamp'], record['motor_at'] = stamp, time.monotonic()
            record['motor_speed'] = max(abs(value) for value in msg.velocity)

    def safety(self, msg, name):
        self.records[name]['safety_active'] = msg.data

    def tick(self):
        now = time.monotonic()
        clock_ready = self.clock_at is not None and now - self.clock_at < self.max_pose_age
        report = {}
        for name, record in self.records.items():
            state = record['state']
            state_stamp = (state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
                           if state is not None else None)
            connected = bool(clock_ready and state and state.connected
                             and now - record['state_at'] < self.state_timeout
                             and -0.05 <= self.sim_time - state_stamp < self.state_timeout)
            pose_age = now - record['pose_at'] if record['pose_at'] is not None else None
            motor_age = now - record['motor_at'] if record['motor_at'] is not None else None
            pose_ready = (clock_ready and record['pose_count'] >= 3 and pose_age < self.max_pose_age
                          and -0.05 <= self.sim_time - record['stamp'] < self.max_pose_age)
            motor_ready = (clock_ready and motor_age is not None and motor_age < self.max_pose_age
                           and -0.05 <= self.sim_time - record['motor_stamp'] < self.max_pose_age)
            service_ready = record['client'].service_is_ready()
            flight_ready = bool(connected and (state.mode in ('TAKEOFF', 'TRACK', 'LAND')
                                               or (state.mode == 'IDLE' and state.landed)))
            checks = {
                'flight': {'ready': flight_ready,
                           'detail': ('connected' if flight_ready else 'waiting for settled ground'
                                      if connected else 'waiting for fresh native flight state')},
                'command service': {'ready': service_ready,
                                    'detail': 'available' if service_ready else 'waiting for FlightCommand service'},
                'ground truth': {'ready': bool(pose_ready),
                                 'detail': 'fresh world odometry' if pose_ready
                                 else f"waiting for fresh odometry ({min(record['pose_count'], 3)}/3)"},
                'motors': {'ready': bool(motor_ready),
                           'detail': 'actual rotor speeds available' if motor_ready
                           else 'waiting for actual rotor JointVelocity telemetry'},
                'safety': {'ready': not record['safety_active'],
                           'detail': 'Gazebo watchdog active' if record['safety_active']
                           else 'Gazebo watchdog clear'},
            }
            ready = all(item['ready'] for item in checks.values())
            report[name] = {
                'connected': connected,
                'telemetry': [topic for topic, available in [('ground_truth/odometry', pose_ready),
                                                            ('motor_speed', motor_ready)] if available],
                'pose_messages': record['pose_count'], 'pose_age_wall_s': pose_age,
                'motor_age_wall_s': motor_age, 'motor_speed_max_rad_s': record['motor_speed'],
                'mode': state.mode if state else None,
                'owner': state.owner if state else None,
                'safety_active': record['safety_active'],
                'armed': bool(state and state.armed), 'landed': bool(state and state.landed),
                'ready': ready, 'checks': checks,
            }
        ready = all(value['ready'] for value in report.values())
        if ready and self.first_ready_elapsed is None:
            self.first_ready_elapsed = now - self.started
        payload = {'ready': ready, 'backend': 'gazebo_native',
                   'updated_wall_unix': time.time(), 'elapsed_wall_s': now - self.started,
                   'first_ready_elapsed_wall_s': self.first_ready_elapsed, 'uavs': report,
                   'simulation': self.simulation_status(now)}
        atomic_json(self.run_dir / 'status/health.json', payload)
        self.publisher.publish(String(data=json.dumps(payload)))
        if ready:
            atomic_json(self.run_dir / 'status/ready.json', payload)
        status_summary = json.dumps({name: item['checks'] for name, item in report.items()}, sort_keys=True)
        if status_summary != self.previous_summary or ready != self.previous_ready or now - self.last_report > 15:
            summary = ', '.join(f"{name}: {'ready' if item['ready'] else 'waiting'} "
                                + '; '.join(f"{check}: {value['detail']}" for check, value in item['checks'].items()
                                            if not value['ready'])
                                for name, item in report.items())
            self.get_logger().info(f"Fleet {'READY' if ready else 'NOT READY'} — {summary}")
            self.previous_ready, self.last_report = ready, now
            self.previous_summary = status_summary


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--fleet', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--max-pose-age', type=float, default=10)
    parser.add_argument('--state-timeout', type=float, default=60)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Readiness(load_config(options.fleet), options.run_dir,
                     options.max_pose_age, options.state_timeout)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
