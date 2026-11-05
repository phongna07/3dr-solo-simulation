#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_bringup'))
from solo_bringup.startup import startup_blockers


def inspect_container(container):
    result = subprocess.run(['docker', 'inspect', container, '--format', '{{json .State}}'],
                            capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(f'Cannot inspect simulation container: {result.stderr.strip()}')
    state = json.loads(result.stdout)
    if state['Status'] in ('exited', 'dead', 'removing'):
        logs = subprocess.run(['docker', 'logs', '--tail', '120', container],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        raise RuntimeError(f"Simulation container {state['Status']} (exit code {state.get('ExitCode')}). "
                           f"Stack stopped before readiness. Recent logs:\n{logs.stdout}")
    if state['Status'] != 'running':
        return float('inf')  # Created/restarting/paused cannot certify live readiness.
    started = state.get('StartedAt', '')
    return datetime.fromisoformat(started.replace('Z', '+00:00')).timestamp() if started else 0


def health_is_ready(data, now, container_started=0):
    updated = data['updated_wall_unix']
    return bool(data['ready'] and updated >= container_started and 0 <= now - updated < 10)


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def process_status(run, startup):
    """A process start is distinct from an MPC constructor finishing."""
    groups = {'RUNNING processes': [], 'READY nodes': [], 'WAIT processes/nodes': [],
              'FAILED processes/nodes': []}
    for name, item in startup.get('processes', {}).items():
        state = item['state']
        if state == 'exited':
            groups['FAILED processes/nodes'].append(f"{name} (exited, code {item['exit_code']})")
        elif state == 'pending':
            groups['WAIT processes/nodes'].append(f'{name} (not started)')
        elif item.get('initializes'):
            component = read_json(run / 'status/components' / f'{name}.json')
            # A resumed/restarted process cannot inherit an old constructor result.
            if component and (component.get('pid') != item.get('pid') or
                              component.get('updated_wall_unix', 0) < startup['started_wall_unix']):
                component = None
            if component and component['state'] == 'initialized':
                groups['READY nodes'].append(name)
            elif component and component['state'] == 'failed':
                groups['FAILED processes/nodes'].append(f"{name} ({component['detail']})")
            else:
                detail = component['detail'] if component else 'loading imports / initializing'
                groups['WAIT processes/nodes'].append(f'{name} ({detail})')
        else:
            groups['RUNNING processes'].append(name)
    return [f"  {label}: {', '.join(names)}" for label, names in groups.items() if names]


def progress_lines(run, data, startup, wall_now, container_started=0):
    lines = []
    if startup and startup.get('started_wall_unix', 0) >= container_started:
        lines.append(f"  Launch: {startup['phase']}")
        lines += process_status(run, startup)
    if not data:
        lines.append('  WAIT readiness monitor: waiting for first health report')
        return lines
    updated = data['updated_wall_unix']
    if updated < container_started or updated > wall_now:
        lines.append('  WAIT readiness monitor: waiting for a health report from this container start')
        return lines
    if wall_now - updated >= 10:
        lines.append('  WAIT readiness monitor: no live health update for at least 10s; checking container')
        return lines
    for name, uav in data['uavs'].items():
        checks = uav['checks']
        ready = [check for check, item in checks.items() if item['ready']]
        waiting = [f"{check} ({item['detail']})" for check, item in checks.items() if not item['ready']]
        if ready:
            lines.append(f"  READY {name}: {', '.join(ready)}")
        if waiting:
            lines.append(f"  WAIT {name}: {'; '.join(waiting)}")
    return lines


def simulation_lines(data):
    """Show live timing on reports without making every clock tick a transition."""
    simulation = data.get('simulation') if isinstance(data, dict) else None
    if not isinstance(simulation, dict):
        return []
    if simulation.get('time_s') is None:
        return ['  Simulation: waiting for Gazebo /clock']
    detail = f"{simulation['time_s']:.1f}s simulated"
    factor = simulation.get('real_time_factor')
    if factor is not None:
        detail += f'; {factor:.2f}x real time (recent window)'
    if simulation.get('clock_age_wall_s', 0) >= 5:
        detail += f"; no clock update for {simulation['clock_age_wall_s']:.0f}s"
    return [f'  Simulation: {detail}']


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id')
    parser.add_argument('--container', help='Docker container ID; detect stopped stack immediately')
    parser.add_argument('--all-processes', action='store_true',
                        help='Also require all launched nodes to finish initialization')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parent.parent / 'runs'
    run = root / (args.run_id or 'latest')
    started = time.monotonic()
    last_report, last_inspect, container_started = None, None, 0
    previous = None
    print('Waiting for simulation readiness (no timeout). '
          'Full logs: ./scripts/dev logs', flush=True)
    while True:
        now = time.monotonic()
        if args.container and (last_inspect is None or now - last_inspect >= 2):
            try:
                container_started = inspect_container(args.container)
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                raise SystemExit(str(error)) from error
            last_inspect = now
        data = read_json(run / 'status/health.json')
        startup = read_json(run / 'status/startup.json')
        try:
            lines = progress_lines(run, data, startup, time.time(), container_started)
            ready = bool(data and health_is_ready(data, time.time(), container_started))
            if args.all_processes:
                blockers = startup_blockers(run, startup)
                ready = ready and not blockers and startup['started_wall_unix'] >= container_started
        except RuntimeError as error:
            raise SystemExit(str(error)) from error
        except (KeyError, TypeError):
            lines = ['  WAIT launch/readiness monitor: waiting for a valid status report']
            ready = False
        summary = '\n'.join(lines)
        if summary != previous or last_report is None or now - last_report >= 15 or ready:
            diagnostics = '\n'.join(simulation_lines(data))
            output = summary + ('\n' + diagnostics if diagnostics else '')
            print(f"[startup +{now - started:.0f}s]\n{output}", flush=True)
            last_report = now
            previous = summary
        if ready:
            detail = 'native flight, measured world odometry and actual rotor telemetry'
            if args.all_processes:
                detail = f'all processes initialized, {detail}'
            print(f'Simulation ready in {now - started:.1f}s ({detail}): {run.resolve()}', flush=True)
            return
        time.sleep(1)


if __name__ == '__main__':
    main()
