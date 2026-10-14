"""Exercise concurrent takeoff and the fleet altitude barrier with a fake FCU."""
from concurrent.futures import Future
import json
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from solo_bringup import flight


class SimulatedFlight:
    wait = flight.Flight.wait
    connected = flight.Flight.connected
    check_response = flight.Flight.check_response
    takeoff = flight.Flight.takeoff
    wait_for_stable_altitude = flight.Flight.wait_for_stable_altitude

    def __init__(self, monkeypatch):
        self.now = 100.0
        self.timeout = 15.0
        self.names = ('uav1', 'uav2', 'uav3')
        self.states = {n: State(connected=True, mode='STABILIZE') for n in self.names}
        self.poses = {}
        self.state_at = {}
        self.pose_at = {}
        self.commands = []
        self.pending = []
        self.delays = {}
        self.rejections = set()
        self.launches = {}
        self.altitudes = lambda n, t: min(3.0, max(0.0, t - self.launches.get(n, t)))
        self.frozen = set()
        self.command_clients = {
            (n, suffix): SimpleNamespace(
                service_is_ready=lambda: True,
                call_async=lambda req, n=n, s=suffix: self.command(n, s, req),
            )
            for n in self.names for suffix in ('set_mode', 'cmd/arming', 'cmd/takeoff')
        }
        monkeypatch.setattr(flight, 'time', SimpleNamespace(monotonic=lambda: self.now))
        monkeypatch.setattr(flight.rclpy, 'ok', lambda: True)
        monkeypatch.setattr(flight.rclpy, 'spin_once', lambda *a, **kw: self.tick())

    def command(self, name, suffix, request):
        self.commands.append((name, suffix, self.now))
        future = Future()
        self.pending.append((self.now + self.delays.get((name, suffix), 0.1),
                             name, suffix, request, future))
        return future

    def tick(self):
        self.now = round(self.now + 0.1, 6)
        for item in list(self.pending):
            deadline, name, suffix, request, future = item
            if self.now < deadline:
                continue
            self.pending.remove(item)
            accepted = (name, suffix) not in self.rejections
            if suffix == 'set_mode':
                response = SetMode.Response(mode_sent=accepted)
                if accepted:
                    self.states[name].mode = request.custom_mode
            elif suffix == 'cmd/arming':
                response = CommandBool.Response(success=accepted, result=0 if accepted else 4)
                if accepted:
                    self.states[name].armed = True
            else:
                response = CommandTOL.Response(success=accepted, result=0 if accepted else 4)
                if accepted:
                    self.launches[name] = self.now
            future.set_result(response)
        for name in self.names:
            self.state_at[name] = self.now
            if name in self.frozen:
                continue
            pose = PoseStamped()
            pose.header.stamp.sec = int(self.now)
            pose.header.stamp.nanosec = round((self.now % 1) * 1e9)
            pose.pose.position.z = self.altitudes(name, self.now)
            self.poses[name] = pose
            self.pose_at[name] = self.now


@pytest.fixture
def fleet(monkeypatch):
    return SimulatedFlight(monkeypatch)


def test_takeoff_launches_fleet_before_waiting_for_altitude(fleet):
    fleet.takeoff(fleet.names, 3.0)
    launches = list(fleet.launches.values())
    assert len(launches) == 3
    assert max(launches) - min(launches) < 0.2
    assert fleet.now >= max(launches) + 5.0


def test_slow_arm_reply_does_not_block_other_takeoffs(fleet):
    fleet.delays[('uav1', 'cmd/arming')] = 4.0
    fleet.takeoff(fleet.names, 3.0)
    assert fleet.launches['uav2'] < fleet.launches['uav1'] - 3.0
    assert fleet.launches['uav3'] < fleet.launches['uav1'] - 3.0
    assert fleet.now >= fleet.launches['uav1'] + 5.0


def test_preflight_checks_entire_fleet_before_arming(fleet):
    fleet.states['uav3'].armed = True
    with pytest.raises(RuntimeError, match='uav3: already armed'):
        fleet.takeoff(fleet.names, 3.0)
    assert not fleet.commands


@pytest.mark.parametrize('suffix', ['set_mode', 'cmd/arming', 'cmd/takeoff'])
def test_rejected_command_aborts_takeoff(fleet, suffix):
    fleet.rejections.add(('uav2', suffix))
    with pytest.raises(RuntimeError, match=f'uav2/{suffix}:'):
        fleet.takeoff(fleet.names, 3.0)


def test_missing_reply_times_out(fleet):
    fleet.delays[('uav2', 'cmd/arming')] = 100.0
    with pytest.raises(RuntimeError, match='Timeout: uav2/cmd/arming'):
        fleet.takeoff(fleet.names, 3.0)
    assert set(fleet.launches) == {'uav1', 'uav3'}


def hover(fleet):
    for state in fleet.states.values():
        state.armed, state.mode = True, 'GUIDED'
    fleet.altitudes = lambda n, t: 3.0


def test_altitude_crossing_while_climbing_is_not_stable(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: 3.0 + 0.3 * (t - 100.0) if n == 'uav3' else 3.0
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_earlier_uav_must_remain_stable_while_last_one_climbs(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: (4.0 if n == 'uav1' and t >= 103.0
                                    else 0.0 if n == 'uav3' and t < 104.0 else 3.0)
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_altitude_departure_resets_settling_interval(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: 4.0 if n == 'uav2' and 101.5 <= t < 102.0 else 3.0
    fleet.wait_for_stable_altitude(fleet.names, 3.0)
    assert fleet.now >= 104.0


def test_frozen_pose_cannot_complete_settling_interval(fleet):
    hover(fleet)
    fleet.tick()
    fleet.frozen.add('uav3')
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


@pytest.mark.parametrize('reaches_altitude', [True, False])
def test_mission_moves_only_after_entire_fleet_settles(
        fleet, monkeypatch, tmp_path, reaches_altitude):
    from solo_mpc import mission

    events = []

    def create_client(typ, name):
        def call(request):
            events.append((name, getattr(request, 'data', None), fleet.now))
            future = Future()
            future.set_result(SimpleNamespace(success=True))
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    def subscribe(typ, topic, callback, qos):
        if topic == '/mpc/status':
            callback(SimpleNamespace(data=json.dumps({
                'reason': 'ready',
                'vehicles': {n: {'result': 'tracking'} for n in fleet.names},
            })))
        return object()

    fleet.create_client = create_client
    fleet.create_subscription = subscribe
    fleet.destroy_node = lambda: None
    fleet.land = lambda names: events.append(('land', None, fleet.now))
    monkeypatch.setattr(mission, 'Flight', lambda *a, **kw: fleet)
    monkeypatch.setattr(mission.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(mission.rclpy, 'shutdown', lambda: None)
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc',
        'config': {'uavs': [{'name': n} for n in fleet.names]},
    }))
    (tmp_path / 'controller.yaml').write_text('flight_altitude: 3.0\ntarget_mode: camera\n')
    (tmp_path / 'scenario.yaml').write_text('{}')
    (tmp_path / 'health.json').write_text(json.dumps({
        'ready': True, 'updated_wall_unix': time.time(),
    }))
    (tmp_path / 'alignment.json').write_text(json.dumps(dict.fromkeys(fleet.names)))
    if reaches_altitude:
        mission.main(['start', '--run-dir', str(tmp_path)])
        starts = [(name, t) for name, enabled, t in events if enabled is True]
        assert [name for name, t in starts] == ['/mpc/enable', '/target/run']
        assert all(t >= max(fleet.launches.values()) + 5.0 for name, t in starts)
    else:
        fleet.altitudes = lambda n, t: 0.0 if n == 'uav3' else 3.0
        with pytest.raises(SystemExit, match='fleet stable'):
            mission.main(['start', '--run-dir', str(tmp_path)])
        assert not any(enabled is True for name, enabled, t in events)
        assert events[-1][0] == 'land'
