#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import time

parser = argparse.ArgumentParser()
parser.add_argument('--run-id')
parser.add_argument('--timeout', type=float, default=600)
args = parser.parse_args()
root = Path(__file__).resolve().parent.parent / 'runs'
run = root / (args.run_id or 'latest')
deadline = time.monotonic() + args.timeout
last_report = 0
while time.monotonic() < deadline:
    try:
        data = json.loads((run / 'health.json').read_text())
        if data['ready'] and time.time() - data['updated_wall_unix'] < 10:
            print(f'Simulation ready: {run.resolve()}', flush=True)
            raise SystemExit(0)
        summary = ', '.join(f"{n}: connected={u['connected']} pose={u['pose_messages']}"
                            for n, u in data['uavs'].items())
    except (OSError, ValueError, KeyError):
        summary = 'waiting for launch/readiness monitor'
    now = time.monotonic()
    if now - last_report > 15:
        print(summary, flush=True)
        last_report = now
    time.sleep(1)
raise SystemExit('Readiness timeout. Stack left running for inspection: ./scripts/dev logs')
