"""Mission commands must reject unsupported runs before initializing ROS."""
import json

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
