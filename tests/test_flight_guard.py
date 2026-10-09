import json
import time

import pytest

pytest.importorskip('rclpy')
from solo_bringup.flight import main


def test_flight_commands_reject_hardware_before_ros_init(tmp_path):
    (tmp_path / 'manifest.json').write_text(json.dumps({'mode': 'hardware'}))
    (tmp_path / 'health.json').write_text(json.dumps({'ready': True, 'updated_wall_unix': time.time()}))
    with pytest.raises(SystemExit, match='simulation-only'):
        main(['takeoff', '--run-dir', str(tmp_path)])


def test_stale_readiness_rejects_takeoff(tmp_path):
    (tmp_path / 'manifest.json').write_text(json.dumps({'mode': 'simulation'}))
    (tmp_path / 'health.json').write_text(json.dumps({'ready': True, 'updated_wall_unix': time.time() - 60}))
    with pytest.raises(SystemExit, match='stale'):
        main(['takeoff', '--run-dir', str(tmp_path)])
