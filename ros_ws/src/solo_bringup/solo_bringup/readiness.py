"""Request telemetry once per connection and monitor actual pose arrivals."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from std_msgs.msg import String

from solo_bringup.configuration import load_config
from solo_bringup.parameters import PluginParameters


def atomic_json(path, payload):
    path = Path(path)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


class Readiness(Node):
    def __init__(self, config, run_dir, timeout, max_pose_age, state_timeout=20.0):
        super().__init__('fleet_readiness')
        self.run_dir = Path(run_dir)
        self.timeout, self.max_pose_age = timeout, max_pose_age
        self.state_timeout = state_timeout
        self.started = time.monotonic()
        self.ever_ready = False
        self.previous_ready = None
        self.previous_summary = None
        self.last_report = 0
        self.failed = False
        self.records = {}
        self.plugin_parameters = PluginParameters(self, config['uavs'], run_dir)
        self.publisher = self.create_publisher(
            String, '/solo/readiness',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions_keepalive = []
        for uav in config['uavs']:
            name = uav['name']
            self.records[name] = {
                'state': None, 'state_at': 0, 'pose_at': 0, 'pose_count': 0,
                'stamp': None, 'intervals': set(), 'future': None, 'request_id': None,
                'next_request': 0, 'request_at': 0,
                'client': self.create_client(MessageInterval, f'/{name}/mavros/set_message_interval'),
            }
            self.subscriptions_keepalive += [
                self.create_subscription(State, f'/{name}/mavros/state',
                                         lambda msg, n=name: self.state(msg, n), qos_profile_sensor_data),
                self.create_subscription(PoseStamped, f'/{name}/mavros/local_position/pose',
                                         lambda msg, n=name: self.pose(msg, n), qos_profile_sensor_data),
            ]
        self.timer = self.create_timer(0.5, self.tick)

    def state(self, msg, name):
        record = self.records[name]
        old = record['state']
        if msg.connected and (old is None or not old.connected):
            self.reset(record)
        elif not msg.connected:
            self.reset(record)
        record['state'], record['state_at'] = msg, time.monotonic()

    @staticmethod
    def reset(record):
        future = record['future']
        if future is not None:
            future.cancel()
        record.update(intervals=set(), future=None, request_id=None, next_request=0,
                      pose_count=0, pose_at=0, stamp=None)

    def pose(self, msg, name):
        values = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                  msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        record = self.records[name]
        if stamp != record['stamp']:
            record['pose_count'] += 1
            record['stamp'] = stamp
            record['pose_at'] = time.monotonic()

    def tick(self):
        now = time.monotonic()
        configured = self.plugin_parameters.tick()
        plugins = self.plugin_parameters.status()
        report = {}
        for name, record in self.records.items():
            state = record['state']
            connected = bool(state and state.connected and now - record['state_at'] < self.state_timeout)
            if not connected:
                self.reset(record)
            elif record['future'] is not None:
                future = record['future']
                if future.done():
                    try:
                        response = future.result()
                        if response and response.success:
                            record['intervals'].add(record['request_id'])
                        else:
                            self.get_logger().warning(f'{name}: telemetry request rejected; retrying')
                    except Exception as error:
                        self.get_logger().warning(f'{name}: telemetry service: {error}')
                    record['future'] = None
                    record['next_request'] = now + 1
                elif now - record['request_at'] > 15:
                    future.cancel()
                    record['future'] = None
                    record['next_request'] = now + 2
            elif now >= record['next_request'] and record['client'].service_is_ready():
                missing = [message for message in (32, 30) if message not in record['intervals']]
                if missing:
                    request = MessageInterval.Request(message_id=missing[0], message_rate=10.0)
                    record['future'] = record['client'].call_async(request)
                    record['request_id'], record['request_at'] = missing[0], now
            pose_age = now - record['pose_at'] if record['pose_at'] else None
            prefix = f'/{name}/mavros/'
            plugin_status = {target.removeprefix(prefix): item for target, item in plugins.items()
                             if target.startswith(prefix)}
            parameters_ready = all(item['ready'] for item in plugin_status.values())
            if not connected:
                connection_detail = 'waiting for FCU heartbeat' if state is None else \
                    ('FCU disconnected' if not state.connected else 'FCU heartbeat stale')
            else:
                connection_detail = 'connected'
            missing = [message for message in (32, 30) if message not in record['intervals']]
            if not missing:
                telemetry_detail = 'position + attitude streams accepted'
            elif not connected:
                telemetry_detail = 'waiting for FCU connection'
            elif record['future'] is not None:
                telemetry_detail = f"waiting for message {record['request_id']} acknowledgement"
            elif not record['client'].service_is_ready():
                telemetry_detail = 'waiting for MAVROS set_message_interval service'
            else:
                telemetry_detail = f'requesting messages {missing}'
            pose_ready = record['pose_count'] >= 3 and pose_age is not None and pose_age < self.max_pose_age
            if not record['pose_count']:
                pose_detail = 'waiting for first local pose (FCU EKF/local_position)'
            elif record['pose_count'] < 3:
                pose_detail = f"collecting fresh poses {record['pose_count']}/3"
            elif not pose_ready:
                pose_detail = 'local pose stream stale'
            else:
                pose_detail = 'fresh local poses'
            checks = {
                'FCU': {'ready': connected, 'detail': connection_detail},
                'MAVROS parameters': {'ready': parameters_ready,
                                      'detail': f"{sum(item['ready'] for item in plugin_status.values())}/{len(plugin_status)} plugins configured"},
                'telemetry': {'ready': not missing, 'detail': telemetry_detail},
                'local pose': {'ready': pose_ready, 'detail': pose_detail},
            }
            ready = configured and connected and record['intervals'] == {32, 30} and \
                record['pose_count'] >= 3 and pose_age is not None and pose_age < self.max_pose_age
            report[name] = {
                'connected': connected, 'telemetry': sorted(record['intervals']),
                'pose_messages': record['pose_count'], 'pose_age_wall_s': pose_age,
                'mode': state.mode if state else None,
                'armed': bool(state and state.armed), 'ready': ready,
                'checks': checks, 'plugins': plugin_status,
            }
        ready = all(value['ready'] for value in report.values())
        payload = {'ready': ready, 'plugin_parameters_applied': configured, 'updated_wall_unix': time.time(),
                   'elapsed_wall_s': now - self.started, 'uavs': report}
        atomic_json(self.run_dir / 'health.json', payload)
        self.publisher.publish(String(data=json.dumps(payload)))
        if ready:
            self.ever_ready = True
            atomic_json(self.run_dir / 'ready.json', payload)
        status_summary = json.dumps({name: {'checks': item['checks'], 'plugins': item['plugins']}
                                     for name, item in report.items()}, sort_keys=True)
        if status_summary != self.previous_summary or ready != self.previous_ready or now - self.last_report > 15:
            summary = ', '.join(f"{name}: {'ready' if item['ready'] else 'waiting'} "
                                + '; '.join(f"{check}: {value['detail']}" for check, value in item['checks'].items()
                                            if not value['ready'])
                                for name, item in report.items())
            self.get_logger().info(f"Fleet {'READY' if ready else 'NOT READY'} — {summary}")
            self.previous_ready, self.last_report = ready, now
            self.previous_summary = status_summary
        if not self.ever_ready and now - self.started > self.timeout:
            self.get_logger().error('Readiness deadline exceeded. Check logs, JSON ports, EKF and RTF.')
            self.failed = True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--fleet', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--max-pose-age', type=float, default=10)
    parser.add_argument('--state-timeout', type=float, default=20)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Readiness(load_config(options.fleet, options.hardware), options.run_dir,
                     options.timeout, options.max_pose_age, options.state_timeout)
    code = 0
    try:
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.5)
        code = 1 if node.failed else 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
