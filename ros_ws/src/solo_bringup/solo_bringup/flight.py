"""Internal takeoff/LAND support for the simulation mission supervisor."""
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


class Flight(Node):
    def __init__(self, uavs, timeout):
        super().__init__('simulation_mission_flight', parameter_overrides=[
            Parameter('use_sim_time', value=True),
        ])
        self.timeout = timeout
        self.states, self.poses, self.state_at, self.pose_at = {}, {}, {}, {}
        self.pose_sim_at = {}
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
        self.pose_sim_at[name] = self.get_clock().now().nanoseconds * 1e-9

    def wait(self, condition, description, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return
        raise RuntimeError(f'Timeout: {description}')

    def connected(self, name):
        state = self.states.get(name)
        return bool(state and state.connected and time.monotonic() - self.state_at[name] < 60)

    def call(self, name, suffix, request):
        client = self.command_clients[(name, suffix)]
        self.wait(client.service_is_ready, f'{name}/{suffix} service', 30)
        future = client.call_async(request)
        self.wait(future.done, f'{name}/{suffix} reply', 30)
        self.check_response(name, suffix, future.result())

    def check_response(self, name, suffix, response):
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
        names = tuple(names)
        self.wait(lambda: all(self.connected(n) and n in self.poses for n in names),
                  'fleet connected + pose')
        # Check the whole fleet before sending any flight commands.
        for name in names:
            if self.states[name].armed:
                raise RuntimeError(f'{name}: already armed; inspect state before takeoff')
        self.wait(lambda: all(self.command_clients[(n, suffix)].service_is_ready()
                             for n in names
                             for suffix in ('set_mode', 'cmd/arming', 'cmd/takeoff')),
                  'fleet takeoff services', 30)

        # Each UAV advances independently on the single ROS executor. In
        # particular, a slow ARM reply cannot delay another UAV's takeoff.
        pending = {}

        def send(name, suffix, request):
            future = self.command_clients[(name, suffix)].call_async(request)
            pending[name] = (suffix, future, time.monotonic() + 30)

        for name in names:
            send(name, 'set_mode', SetMode.Request(base_mode=0, custom_mode='GUIDED'))

        def advance():
            for name, (phase, future, deadline) in list(pending.items()):
                if phase == 'GUIDED':
                    if self.connected(name) and self.states[name].mode == 'GUIDED':
                        send(name, 'cmd/arming', CommandBool.Request(value=True))
                        continue
                elif future.done():
                    self.check_response(name, phase, future.result())
                    if phase == 'set_mode':
                        pending[name] = ('GUIDED', None, time.monotonic() + self.timeout)
                    elif phase == 'cmd/arming':
                        # No fleet barrier between ARM ACK and takeoff:
                        # ArduPilot may auto-disarm vehicles left on the ground.
                        send(name, 'cmd/takeoff', CommandTOL.Request(altitude=altitude))
                    else:
                        del pending[name]
                    continue
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Timeout: {name}/{phase}')
            return not pending

        self.wait(advance, 'fleet takeoff commands', self.timeout + 90)
        self.wait_for_stable_altitude(names, altitude)
        print('All selected UAVs stable at takeoff altitude.', flush=True)

    def wait_for_stable_altitude(self, names, altitude):
        """Require two seconds of fresh, settled telemetry for every UAV.

        Gazebo receipt timestamps measure vertical speed and the settling
        interval. MAVROS stamps are in a different clock domain; only use them
        to reject repeated samples. Wall deadlines still detect a stalled sim.
        """
        previous, stable_since = {}, {}

        def settled():
            ready = True
            for name in names:
                pose = self.poses.get(name)
                if not (self.connected(name) and self.states[name].armed
                        and self.states[name].mode == 'GUIDED' and pose is not None
                        and time.monotonic() - self.pose_at.get(name, 0) < 3):
                    previous.pop(name, None)
                    stable_since.pop(name, None)
                    ready = False
                    continue
                stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
                sim = self.pose_sim_at[name]
                z = pose.pose.position.z
                old = previous.get(name)
                if old is None or stamp != old[0]:
                    previous[name] = (stamp, sim, z)
                    if (old is not None and 0 < sim - old[1] <= 1
                            and abs(z - altitude) <= 0.5
                            and abs(old[2] - altitude) <= 0.5
                            and abs((z - old[2]) / (sim - old[1])) <= 0.2):
                        stable_since.setdefault(name, old[1])
                    else:
                        stable_since.pop(name, None)
                if name not in stable_since or previous[name][1] - stable_since[name] < 2:
                    ready = False
            return ready

        self.wait(settled, f'fleet stable at {altitude} m')

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
