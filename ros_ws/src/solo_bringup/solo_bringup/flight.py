"""Internal takeoff/LAND support for the simulation mission supervisor."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


class Flight(Node):
    def __init__(self, uavs, timeout):
        super().__init__('simulation_mission_flight')
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
