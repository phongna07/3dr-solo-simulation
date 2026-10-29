import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
from datetime import datetime, timezone
from uuid import uuid4

import yaml


def sitl_parameter_file(parameters, destination):
    """Shared read-only defaults; each SITL still owns its own EEPROM directory."""
    if not isinstance(parameters, dict):
        raise ValueError('sitl_params must be a mapping of FCU parameter names to numbers')
    lines = []
    for name, value in parameters.items():
        if (not isinstance(name, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]{0,15}', name)
                or type(value) not in (int, float) or not math.isfinite(value)):
            raise ValueError(f'Invalid SITL parameter: {name}')
        lines.append(f'{name} {value}\n')
    Path(destination).write_text(''.join(lines))
    return str(destination)


def create_run(config, run_id=''):
    root = Path(os.environ.get('RUNS_ROOT', '/workspace/runs'))
    run_id = run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid4().hex[:8]
    if run_id == 'latest' or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', run_id):
        raise ValueError('Invalid run_id')
    root.mkdir(parents=True, exist_ok=True)
    run = root / run_id
    if run.exists():
        raise ValueError(f'Run exists: {run}; select a fresh run_id')
    run.mkdir()
    for directory in ('config', 'metadata', 'status', 'telemetry', 'reports'):
        (run / directory).mkdir()
    versions_path = Path('/opt/solo/versions.json')
    manifest = {
        'run_id': run_id, 'mode': 'simulation', 'profile': 'mpc', 'config': config,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'rmw': os.environ.get('RMW_IMPLEMENTATION', ''),
        'clock_policy': 'wall; FCU physics follows Gazebo in simulation',
        'application_mode': 'development' if os.environ.get('SOLO_DEV') == '1' else 'release',
        'versions': json.loads(versions_path.read_text()) if versions_path.exists() else {},
    }
    try:
        manifest['workspace_commit'] = subprocess.check_output(
            ['git', '-C', '/workspace', 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL,
            text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        manifest['workspace_commit'] = None
    (run / 'metadata/manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (run / 'config/fleet.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    for filename in ('apt-packages.lock', 'submodules.lock', 'geoid.sha256', 'application.sha256'):
        source = Path('/opt/solo') / filename
        if source.exists():
            shutil.copy2(source, run / 'metadata' / filename)
    hashes = {}
    for source_root in ('/workspace/config', '/workspace/ros_ws/src/solo_bringup',
                        '/workspace/ros_ws/src/solo_mpc'):
        for file in sorted(Path(source_root).rglob('*')):
            if file.is_file() and '__pycache__' not in file.parts:
                hashes[str(file.relative_to('/workspace'))] = hashlib.sha256(file.read_bytes()).hexdigest()
    (run / 'metadata/source-sha256.json').write_text(json.dumps(hashes, indent=2) + '\n')
    logs = run / 'logs'
    logs.mkdir(exist_ok=True)
    # Atomic symlink replacement; no state directories are deleted.
    temp = root / f'.latest-{uuid4().hex}'
    temp.symlink_to(run_id, target_is_directory=True)
    temp.replace(root / 'latest')
    return run
