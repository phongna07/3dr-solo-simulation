"""Mission progress is visible before completion, including without a terminal."""

from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def cli(monkeypatch, tmp_path):
    # Exercise the real CLI loop without requiring ROS or flying vehicles.
    modules = {}
    for name in ('rclpy', 'rclpy.clock', 'rclpy.node', 'rclpy.signals',
                 'solo_mpc_interfaces', 'solo_mpc_interfaces.srv',
                 'std_srvs', 'std_srvs.srv'):
        modules[name] = ModuleType(name)
        monkeypatch.setitem(sys.modules, name, modules[name])
    modules['rclpy.clock'].Clock = SimpleNamespace
    modules['rclpy.clock'].ClockType = SimpleNamespace(STEADY_TIME=1)
    modules['rclpy.node'].Node = SimpleNamespace
    modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO=0)
    service = SimpleNamespace(Request=SimpleNamespace)
    modules['solo_mpc_interfaces.srv'].StartMission = service
    modules['std_srvs.srv'].Trigger = service

    spec = importlib.util.spec_from_file_location(
        'mission_cli_under_test', Path(__file__).resolve().parents[1]
        / 'ros_ws/src/solo_mpc/solo_mpc/mission.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc',
    }))
    state = SimpleNamespace(snapshots=[], ready_at=0, destroyed=False, shutdown=False)

    def snapshot(name, owner='unowned', **kwargs):
        return dict(state=name, mission_id='test-mission', run_dir=str(tmp_path),
                    ownership={'uav1': owner}, controller={'reason': 'ready'},
                    error='', **kwargs)

    def client(service, path):
        def call(request):
            future = Future()
            if path == '/mission/status':
                # Keep the final snapshot available for repeated queries.
                value = state.snapshots[0]
                if len(state.snapshots) > 1:
                    state.snapshots.pop(0)
                message = json.dumps(value)
            else:
                message = 'accepted'
            future.set_result(SimpleNamespace(
                success=True, mission_id='test-mission', message=message))
            return future
        return SimpleNamespace(service_is_ready=lambda: clock.now >= state.ready_at,
                               call_async=call)

    def spin(*args, **kwargs):
        clock.now += 1.0

    def destroy():
        state.destroyed = True

    def shutdown():
        state.shutdown = True

    node = SimpleNamespace(create_client=client, destroy_node=destroy)
    monkeypatch.setattr(module, 'Node', lambda name: node)
    ros = modules['rclpy']
    ros.init = lambda **kwargs: None
    ros.ok = lambda: True
    ros.spin_once = spin
    ros.shutdown = shutdown

    def run(command='run'):
        return module.main([command, '--run-dir', str(tmp_path)])

    return SimpleNamespace(run=run, state=state, snapshot=snapshot, module=module)


def test_progress_reports_takeoff_tracking_and_landing_before_completion(cli, capsys):
    cli.state.snapshots = [cli.snapshot('starting'), cli.snapshot('starting', 'guided'),
                           cli.snapshot('tracking', 'owned'),
                           cli.snapshot('landing', 'landing'), cli.snapshot('completed')]
    cli.run()
    output = capsys.readouterr()
    assert 'Mission completed and fleet landed.' in output.out
    assert 'Requesting mission start' in output.err
    assert 'Mission accepted (test-mission)' in output.err
    messages = ['Mission starting; uav1=unowned', 'Mission starting; uav1=guided',
                'Mission tracking; uav1=owned', 'Mission landing; uav1=landing',
                'Mission completed; uav1=unowned']
    positions = [output.err.index(message) for message in messages]
    assert positions == sorted(positions)
    assert cli.state.destroyed and cli.state.shutdown


def test_unchanged_state_has_periodic_heartbeat_without_poll_spam(cli, capsys):
    cli.state.snapshots = [cli.snapshot('tracking', 'owned') for _ in range(7)]
    cli.state.snapshots.append(cli.snapshot('completed'))
    cli.run()
    output = capsys.readouterr()
    # Each status call advances two fake wall seconds; heartbeats are >=5s apart.
    assert output.err.count('Mission tracking;') == 3


def test_service_discovery_reports_long_waits(cli, capsys):
    cli.state.ready_at = 12
    cli.state.snapshots = [cli.snapshot('completed')]
    cli.run()
    assert capsys.readouterr().err.count('Waiting for mission supervisor start service') == 2


def test_status_remains_machine_readable_without_progress(cli, capsys):
    expected = cli.snapshot('tracking', 'owned')
    cli.state.snapshots = [expected]
    cli.run('status')
    output = capsys.readouterr()
    assert json.loads(output.out) == expected
    assert output.err == ''


def test_progress_is_flushed_while_mission_is_still_running(cli, monkeypatch):
    cli.state.snapshots = [cli.snapshot('tracking', 'owned'), cli.snapshot('completed')]
    writes = []

    def record(*args, **kwargs):
        writes.append((args, kwargs))
        if args[0].endswith('Mission tracking; uav1=owned; controller: ready'):
            assert not cli.state.destroyed
            assert cli.state.snapshots[0]['state'] == 'completed'

    monkeypatch.setattr(cli.module, 'print', record, raising=False)
    cli.run()
    progress = [kwargs for args, kwargs in writes if kwargs.get('file') is sys.stderr]
    assert len(progress) >= 4
    assert all(kwargs.get('flush') is True for kwargs in progress)


def test_failure_still_reports_error_and_cleans_up_client(cli, capsys):
    failed = cli.snapshot('failed')
    failed['error'] = 'Controller status stopped updating'
    cli.state.snapshots = [failed]
    with pytest.raises(SystemExit, match='Controller status stopped updating'):
        cli.run()
    assert 'Mission failed;' in capsys.readouterr().err
    assert cli.state.destroyed and cli.state.shutdown
