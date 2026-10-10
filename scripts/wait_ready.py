#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time


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


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id')
    parser.add_argument('--container', help='Docker container ID; detect stopped stack immediately')
    parser.add_argument('--timeout', type=float, default=600)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parent.parent / 'runs'
    run = root / (args.run_id or 'latest')
    deadline = time.monotonic() + args.timeout
    last_report, last_inspect, container_started = 0, None, 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if args.container and (last_inspect is None or now - last_inspect >= 2):
            try:
                container_started = inspect_container(args.container)
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                raise SystemExit(str(error)) from error
            last_inspect = now
        try:
            data = json.loads((run / 'health.json').read_text())
            if health_is_ready(data, time.time(), container_started):
                print(f'Simulation ready: {run.resolve()}', flush=True)
                return
            age = time.time() - data['updated_wall_unix']
            if age >= 10:
                summary = f'readiness monitor has not updated for {age:.0f}s; checking container'
            else:
                summary = ', '.join(f"{n}: connected={u['connected']} pose={u['pose_messages']}"
                                    for n, u in data['uavs'].items())
        except (OSError, ValueError, KeyError):
            summary = 'waiting for launch/readiness monitor'
        if now - last_report > 15:
            print(summary, flush=True)
            last_report = now
        time.sleep(1)
    raise SystemExit('Readiness timeout. Stack left running for inspection: ./scripts/dev logs')


if __name__ == '__main__':
    main()
