"""Explicit simulation-only flight operations; verify state beyond service ACKs."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


class Flight(Node):
    def __init__(self, uavs, timeout):
        super().__init__('simulation_flight_cli')
        self.timeout = timeout
        self.states, self.poses, self.state_at, self.pose_at = {}, {}, {}, {}
        self.subscriptions_keepalive = []
        self.command_clients = {}
        for uav in uavs:
            name = uav['name']
            self.subscriptions_keepalive += [
                self.create_subscription(State, f'/{name}/mavros/state',
                                         lambda msg, n=name: self.state(msg, n), qos_profile_sensor_data),
                self.create_subscription(PoseStamped, f'/{name}/mavros/local_position/pose',
                                         lambda msg, n=name: self.pose(msg, n), qos_profile_sensor_data),
            ]
            for suffix, service in (('set_mode', SetMode), ('cmd/arming', CommandBool),
                                    ('cmd/takeoff', CommandTOL)):
                self.command_clients[(name, suffix)] = self.create_client(service, f'/{name}/mavros/{suffix}')

    def state(self, msg, name):
        self.states[name], self.state_at[name] = msg, time.monotonic()

    def pose(self, msg, name):
        self.poses[name], self.pose_at[name] = msg, time.monotonic()

    def wait(self, condition, description, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return
        raise RuntimeError(f'Timeout: {description}')

    def connected(self, name):
        state = self.states.get(name)
        return bool(state and state.connected and time.monotonic() - self.state_at[name] < 20)

    def call(self, name, suffix, request):
        client = self.command_clients[(name, suffix)]
        self.wait(client.service_is_ready, f'{name}/{suffix} service', 30)
        future = client.call_async(request)
        self.wait(future.done, f'{name}/{suffix} reply', 30)
        response = future.result()
        if response is None:
            raise RuntimeError(f'{name}/{suffix}: no response')
        if hasattr(response, 'success') and (not response.success or response.result != 0):
            raise RuntimeError(f'{name}/{suffix}: rejected {response}')
        if hasattr(response, 'mode_sent') and not response.mode_sent:
            raise RuntimeError(f'{name}/{suffix}: mode request not sent')
        print(f'{name}/{suffix}: {response}', flush=True)

    def mode(self, name, mode):
        self.call(name, 'set_mode', SetMode.Request(base_mode=0, custom_mode=mode))
        self.wait(lambda: self.connected(name) and self.states[name].mode == mode,
                  f'{name}: actual mode {mode}')

    def takeoff(self, names, altitude):
        for name in names:
            self.wait(lambda: self.connected(name) and name in self.poses, f'{name}: connected + pose')
            if self.states[name].armed:
                raise RuntimeError(f'{name}: already armed; inspect state before takeoff')
            self.mode(name, 'GUIDED')
            self.call(name, 'cmd/arming', CommandBool.Request(value=True))
            # No long wait between ARM ACK and takeoff (auto-disarm on ground).
            self.call(name, 'cmd/takeoff', CommandTOL.Request(altitude=altitude))
            self.wait(lambda: self.connected(name) and self.states[name].armed
                      and self.states[name].mode == 'GUIDED'
                      and time.monotonic() - self.pose_at.get(name, 0) < 10
                      and self.poses[name].pose.position.z >= altitude - 0.5,
                      f'{name}: observed takeoff to {altitude} m')
            print(f'{name}: takeoff observed', flush=True)

    def land(self, names):
        # Send LAND to every connected UAV before waiting for any one to disarm.
        errors = []
        for name in names:
            try:
                self.wait(lambda: self.connected(name), f'{name}: connection', 20)
                self.call(name, 'set_mode', SetMode.Request(base_mode=0, custom_mode='LAND'))
            except RuntimeError as error:
                errors.append(str(error))
        for name in names:
            try:
                self.wait(lambda: self.connected(name) and not self.states[name].armed,
                          f'{name}: landed/disarmed')
            except RuntimeError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('; '.join(errors))
        print('All selected UAVs disarmed.', flush=True)

    def stream(self, names, command, seconds, xyz):
        self.wait(lambda: all(self.connected(n) and self.states[n].armed
                             and self.states[n].mode == 'GUIDED' for n in names),
                  'selected UAVs armed in GUIDED', 20)
        msg_type = TwistStamped if command == 'velocity' else PoseStamped
        suffix = 'setpoint_velocity/cmd_vel' if command == 'velocity' else 'setpoint_position/local'
        publishers = {n: self.create_publisher(msg_type, f'/{n}/mavros/{suffix}', 10) for n in names}
        self.wait(lambda: all(p.get_subscription_count() for p in publishers.values()),
                  'setpoint subscribers', 20)

        def publish(values):
            for name, publisher in publishers.items():
                msg = msg_type()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'{name}/map'
                if command == 'velocity':
                    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = values
                else:
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = values
                    msg.pose.orientation.w = 1.0
                publisher.publish(msg)

        try:
            deadline = time.monotonic() + seconds
            next_tick = time.monotonic()
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0)
                if not all(self.connected(n) and self.states[n].armed
                           and self.states[n].mode == 'GUIDED' for n in names):
                    raise RuntimeError('Lost GUIDED/armed/connection while publishing')
                publish(xyz)
                next_tick += 0.1
                time.sleep(max(0, next_tick - time.monotonic()))
        finally:
            if command == 'velocity' and rclpy.ok():
                # Best effort stop even after Ctrl+C or a stream error.
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    publish((0.0, 0.0, 0.0))
                    time.sleep(0.1)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'takeoff', 'land', 'velocity', 'position'])
    parser.add_argument('--run-dir', default='/workspace/runs/latest')
    parser.add_argument('--uav', help='One configured name; defaults to all (position requires one)')
    parser.add_argument('--altitude', type=float, default=3)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--xyz', type=float, nargs=3, default=[0.5, 0.0, 0.0])
    parser.add_argument('--timeout', type=float, default=240)
    options = parser.parse_args(args)
    run = Path(options.run_dir)
    manifest = json.loads((run / 'manifest.json').read_text())
    health = json.loads((run / 'health.json').read_text())
    if options.command == 'status':
        print(json.dumps(health, indent=2))
        return
    if manifest['mode'] != 'simulation':
        raise SystemExit('This flight helper is simulation-only. Hardware requires a reviewed controller.')
    if options.command != 'land' and (not health['ready'] or time.time() - health['updated_wall_unix'] > 10):
        raise SystemExit('Fleet is not ready or health is stale. Inspect logs/status first.')
    uavs = manifest['config']['uavs']
    if options.uav:
        uavs = [u for u in uavs if u['name'] == options.uav]
    if not uavs or (options.command == 'position' and len(uavs) != 1):
        raise SystemExit('Select a configured UAV with --uav (required for position).')
    if not all(math.isfinite(x) for x in [options.altitude, options.seconds, options.timeout, *options.xyz]) \
            or options.altitude <= 0 or options.seconds <= 0 or options.timeout <= 0:
        raise SystemExit('Numeric arguments must be finite; altitude, seconds and timeout positive.')
    # Keep the context alive during Python's SIGINT so finally can publish zero velocity.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Flight(uavs, options.timeout)
    names = [u['name'] for u in uavs]
    try:
        if options.command == 'takeoff':
            node.takeoff(names, options.altitude)
        elif options.command == 'land':
            node.land(names)
        else:
            node.stream(names, options.command, options.seconds, options.xyz)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
