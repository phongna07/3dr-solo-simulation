"""Mission commands must reject unsupported runs before initializing ROS."""
import json
import time

import pytest

pytest.importorskip("rclpy")
from solo_mpc.mission import main


@pytest.mark.parametrize("mode,profile", [("hardware", "mpc"), ("simulation", "control")])
@pytest.mark.parametrize("command", ["run", "start", "stop", "status", "reset"])
def test_mission_rejects_non_mission_runs(tmp_path, mode, profile, command):
    (tmp_path / "manifest.json").write_text(json.dumps({"mode": mode, "profile": profile}))
    with pytest.raises(SystemExit, match="MPC simulation profile"):
        main([command, "--run-dir", str(tmp_path)])


@pytest.mark.parametrize("seconds", ["-1", "nan", "inf"])
def test_mission_rejects_invalid_duration(seconds):
    with pytest.raises(SystemExit) as exc:
        main(["run", "--seconds", seconds])
    assert exc.value.code == 2


def test_startup_can_be_cancelled_before_run_directory_exists(tmp_path):
    cancel = tmp_path / 'mission.cancel'
    cancel.touch()
    main(['run', '--run-dir', str(tmp_path / 'not-created'),
          '--wait-for-startup', '--cancel-file', str(cancel)])


def test_cancellation_works_while_simulation_clock_is_paused(tmp_path, capsys):
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc', 'config': {'uavs': []},
    }))
    (tmp_path / 'controller.yaml').write_text('{}')
    (tmp_path / 'scenario.yaml').write_text('{}')
    cancel = tmp_path / 'mission.cancel'
    cancel.touch()
    started = time.monotonic()
    # Flight's /clock remains zero. Cancellation must use a steady timer,
    # including while waiting for a service that never becomes available.
    main(['run', '--run-dir', str(tmp_path), '--cancel-file', str(cancel)])
    assert time.monotonic() - started < 3
    assert 'Interrupted; stopping target' in capsys.readouterr().out
