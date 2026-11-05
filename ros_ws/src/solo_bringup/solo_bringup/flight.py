"""Owner-bound native flight support for the simulation mission supervisor."""
import math
import time
from enum import Enum
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from solo_mpc_interfaces.msg import FlightState
from solo_mpc_interfaces.srv import FlightCommand


class Ownership(str, Enum):
    UNOWNED = 'unowned'
    ACQUIRING = 'acquiring'
    ACQUIRED = 'acquired'
    OWNED = 'owned'
    LAND_REQUESTED = 'land_requested'
    LANDING = 'landing'
    RELEASED = 'released'


class Flight(Node):
    def __init__(self, uavs, timeout, settings=None):
        super().__init__('simulation_mission_flight', parameter_overrides=[
            Parameter('use_sim_time', value=True),
        ])
        self.timeout = timeout
        self.flight_settings = settings or {}
        self.owner_token = ''
        self.states, self.poses, self.state_at, self.pose_at = {}, {}, {}, {}
        self.pose_sim_at, self.velocities = {}, {}
        self.landing_status = {}
        self.subscriptions_keepalive = []
        self.command_clients = {}
        self.ownership = {u['name']: Ownership.UNOWNED for u in uavs}
        for uav in uavs:
            name = uav['name']
            self.subscriptions_keepalive += [
                self.create_subscription(FlightState, f'/{name}/flight/state',
                                         lambda msg, n=name: self.state(msg, n), qos_profile_sensor_data),
                self.create_subscription(Odometry, f'/{name}/ground_truth/odometry',
                                         lambda msg, n=name: self.odometry(msg, n), qos_profile_sensor_data),
            ]
            self.command_clients[name] = self.create_client(FlightCommand, f'/{name}/flight/command')

    def state(self, msg, name):
        self.states[name], self.state_at[name] = msg, time.monotonic()
        self.observe_ownership(name)

    @property
    def flight_owned(self):
        return any(owner not in (Ownership.UNOWNED, Ownership.RELEASED)
                   for owner in self.ownership.values())

    def observe_ownership(self, name):
        """Latch takeover; never send LAND to a different owner's vehicle."""
        state = self.states.get(name)
        owner = self.ownership[name]
        if state is None or owner in (Ownership.UNOWNED, Ownership.RELEASED):
            return
        if state.owner and state.owner != self.owner_token:
            owner = Ownership.RELEASED
        elif not state.owner:
            if (owner in (Ownership.LAND_REQUESTED, Ownership.LANDING)
                    and not state.armed and state.landed and state.mode == 'IDLE'):
                owner = Ownership.UNOWNED
            elif owner != Ownership.ACQUIRING or state.mode != 'IDLE' or state.armed:
                owner = Ownership.RELEASED
        elif state.mode == 'LAND':
            owner = Ownership.LANDING
        elif owner in (Ownership.LAND_REQUESTED, Ownership.LANDING):
            if owner == Ownership.LANDING or state.mode not in ('IDLE', 'TAKEOFF', 'TRACK'):
                owner = Ownership.RELEASED
        elif state.mode not in ('IDLE', 'TAKEOFF', 'TRACK'):
            owner = Ownership.RELEASED
        elif owner == Ownership.OWNED and (not state.armed or state.mode == 'IDLE'):
            owner = Ownership.RELEASED
        elif state.armed and state.mode in ('TAKEOFF', 'TRACK'):
            owner = Ownership.OWNED
        else:
            owner = Ownership.ACQUIRED
        self.ownership[name] = owner

    def check_ownership(self):
        for name in self.ownership:
            self.observe_ownership(name)
        released = [name for name, owner in self.ownership.items() if owner == Ownership.RELEASED]
        if released:
            raise FlightReleased('Native flight owner/mode takeover: ' + ', '.join(released))
        landing = [name for name, owner in self.ownership.items() if owner == Ownership.LANDING]
        if landing:
            raise RuntimeError('Native flight entered failsafe LAND: ' + ', '.join(landing))

    def odometry(self, msg, name):
        previous = self.poses.get(name)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        if (msg.header.frame_id != 'world'
                or not all(math.isfinite(value) for value in (
                    position.x, position.y, position.z, orientation.x, orientation.y,
                    orientation.z, orientation.w, linear.x, linear.y, linear.z))
                or (previous is not None and stamp <= self.pose_sim_at[name])):
            return
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.poses[name], self.pose_at[name] = pose, time.monotonic()
        self.pose_sim_at[name] = stamp
        self.velocities[name] = msg.twist.twist.linear

    def wait(self, condition, description, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return
        raise RuntimeError(f'Timeout: {description}')

    def connected(self, name):
        state = self.states.get(name)
        if state is None or not state.connected:
            return False
        stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        sim = self.get_clock().now().nanoseconds * 1e-9
        return (-0.05 <= sim - stamp <= self.flight_settings.get('flight_timeout_sim', 2.5)
                and time.monotonic() - self.state_at[name]
                <= self.flight_settings.get('flight_timeout_wall', 60.0))

    def check_response(self, name, command, response):
        if response is None or not response.success:
            raise RuntimeError(f'{name}/{command}: {response.message if response else "no response"}')
        print(f'{name}/{command}: {response.message}', flush=True)

    def takeoff(self, names, altitude):
        names = tuple(names)
        self.wait(lambda: all(self.connected(name) and name in self.poses for name in names),
                  'fleet connected + measured odometry')
        for name in names:
            state = self.states[name]
            if state.armed or state.owner or not state.landed or state.mode != 'IDLE':
                raise RuntimeError(f'{name}: not unowned, settled, disarmed IDLE flight')
        self.wait(lambda: all(self.command_clients[name].service_is_ready() for name in names),
                  'fleet native flight services', 30)
        self.check_ownership()
        self.owner_token = uuid.uuid4().hex
        pending = {}

        def send(name, command):
            self.check_ownership()
            request = FlightCommand.Request(owner=self.owner_token, command=command, altitude=altitude)
            future = self.command_clients[name].call_async(request)
            pending[name] = (command, future, time.monotonic() + 30)

        for name in names:
            self.ownership[name] = Ownership.ACQUIRING
            send(name, 'acquire')

        def advance():
            self.check_ownership()
            for name, (phase, future, deadline) in list(pending.items()):
                if future.done():
                    self.check_response(name, phase, future.result())
                    if phase == 'acquire':
                        send(name, 'arm')
                    elif phase == 'arm':
                        send(name, 'takeoff')
                    else:
                        del pending[name]
                elif time.monotonic() >= deadline:
                    future.cancel()
                    raise RuntimeError(f'Timeout: {name}/{phase}')
            return not pending

        try:
            self.wait(advance, 'fleet takeoff commands', self.timeout + 90)
        finally:
            for _, future, _ in pending.values():
                future.cancel()
        self.wait_for_stable_altitude(names, altitude)
        print('All selected UAVs stable at takeoff altitude.', flush=True)

    def wait_for_stable_altitude(self, names, altitude):
        """Require two simulation seconds of fresh measured pose and velocity."""
        stable_since, previous_stamp = {}, {}

        def settled():
            self.check_ownership()
            ready = True
            sim = self.get_clock().now().nanoseconds * 1e-9
            for name in names:
                pose, velocity = self.poses.get(name), self.velocities.get(name)
                stamp = self.pose_sim_at.get(name, 0.0)
                fresh = (self.connected(name) and self.states[name].armed
                         and self.states[name].mode == 'TAKEOFF' and pose is not None
                         and velocity is not None
                         and -0.05 <= sim - stamp <= self.flight_settings.get('state_timeout_sim', 0.5)
                         and time.monotonic() - self.pose_at.get(name, 0)
                         <= self.flight_settings.get('pose_timeout_wall', 3.0))
                speed = (sum(value**2 for value in (velocity.x, velocity.y, velocity.z))**0.5
                         if velocity is not None else float('inf'))
                if (name in previous_stamp and stamp - previous_stamp[name]
                        > self.flight_settings.get('state_timeout_sim', 0.5)):
                    stable_since.pop(name, None)
                previous_stamp[name] = stamp
                if fresh and abs(pose.pose.position.z - altitude) <= 0.5 and speed <= 0.2:
                    stable_since.setdefault(name, stamp)
                else:
                    stable_since.pop(name, None)
                if name not in stable_since or stamp - stable_since[name] < 2.0:
                    ready = False
            return ready

        self.wait(settled, f'fleet stable at world ENU z={altitude} m')

    def track(self, names):
        pending = {}
        self.check_ownership()
        for name in names:
            if self.states[name].owner != self.owner_token:
                raise FlightReleased(f'{name}: flight owner changed before TRACK')
            pending[name] = self.command_clients[name].call_async(FlightCommand.Request(
                owner=self.owner_token, command='track'))

        def accepted():
            self.check_ownership()
            for name, future in list(pending.items()):
                if future.done():
                    self.check_response(name, 'track', future.result())
                    del pending[name]
            return not pending

        try:
            self.wait(accepted, 'fleet TRACK commands', 30.0)
        finally:
            for future in pending.values():
                future.cancel()

    def land(self, names, *, poll=lambda: None, command_timeout=5.0,
             landing_timeout_sim=60.0, stall_timeout_wall=60.0):
        """Dispatch owner-bound LAND independently, then confirm settled disarm."""
        names = tuple(names)
        pending = {name for name in names if self.ownership[name]
                   not in (Ownership.UNOWNED, Ownership.RELEASED)}
        sent, errors = {}, []
        started = time.monotonic()
        command_deadline = started + command_timeout
        sim_started = last_sim = self.get_clock().now().nanoseconds * 1e-9
        clock_progress_at = started
        while rclpy.ok():
            poll()
            now = time.monotonic()
            sim = self.get_clock().now().nanoseconds * 1e-9
            if sim > last_sim:
                clock_progress_at = now
            elif sim < last_sim:
                errors.append('Simulation clock moved backwards during landing')
                break
            last_sim = sim
            for name in tuple(pending):
                self.observe_ownership(name)
                if self.ownership[name] in (Ownership.UNOWNED, Ownership.RELEASED):
                    pending.remove(name)
                    continue
                if now >= command_deadline:
                    errors.append(f'Timeout: {name}/land service')
                    pending.remove(name)
                    continue
                client = self.command_clients[name]
                if client.service_is_ready():
                    self.ownership[name] = Ownership.LAND_REQUESTED
                    try:
                        sent[name] = client.call_async(FlightCommand.Request(
                            owner=self.owner_token, command='land'))
                    except Exception as exc:
                        errors.append(f'{name}/land: {exc}')
                    pending.remove(name)
            for name, future in list(sent.items()):
                self.observe_ownership(name)
                if self.ownership[name] == Ownership.RELEASED:
                    future.cancel()
                    del sent[name]
                elif future.done():
                    try:
                        self.check_response(name, 'land', future.result())
                    except Exception as exc:
                        errors.append(str(exc))
                    del sent[name]
                elif now >= command_deadline:
                    errors.append(f'Timeout: {name}/land reply')
                    future.cancel()
                    del sent[name]
            airborne = []
            for name in names:
                self.observe_ownership(name)
                if self.ownership[name] not in (Ownership.UNOWNED, Ownership.RELEASED):
                    airborne.append(name)
            self.landing_status = {
                'elapsed_sim': sim - sim_started, 'elapsed_wall': now - started,
                'timeout_sim': landing_timeout_sim,
                'clock_stalled_wall': now - clock_progress_at,
                'vehicles': {
                    name: {
                        'ownership': self.ownership[name].value,
                        'owner': self.states[name].owner if name in self.states else None,
                        'armed': self.states[name].armed if name in self.states else None,
                        'landed': self.states[name].landed if name in self.states else None,
                        'mode': self.states[name].mode if name in self.states else None,
                        'state_age_wall': now - self.state_at.get(name, started),
                        'altitude': self.poses[name].pose.position.z if name in self.poses else None,
                    } for name in names
                },
            }
            if not pending and not sent and not airborne:
                break
            if sim - sim_started >= landing_timeout_sim:
                errors.extend(f'Timeout: {name}: landed/disarmed after '
                              f'{sim - sim_started:.1f} simulation seconds' for name in airborne)
                break
            if now - clock_progress_at >= stall_timeout_wall:
                errors.append(f'Timeout: simulation clock stalled for {stall_timeout_wall:g} wall seconds')
                break
            stale = [name for name in airborne
                     if now - self.state_at.get(name, started) >= stall_timeout_wall]
            if stale:
                errors.extend(f'Timeout: {name}: native flight state stale for '
                              f'{stall_timeout_wall:g} wall seconds' for name in stale)
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            errors.append('ROS stopped during landing')
        for future in sent.values():
            future.cancel()
        if errors:
            raise RuntimeError('; '.join(errors))


class FlightReleased(Exception):
    """Another owner or mode has taken flight; the mission must not reclaim it."""
