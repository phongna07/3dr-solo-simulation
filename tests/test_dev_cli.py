"""Check mission launch wiring and shutdown order without starting vehicles."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def fake_docker(tmp_path):
    log = tmp_path / "docker.jsonl"
    docker = tmp_path / "docker"
    docker.write_text(f"#!{sys.executable}\n" + '''import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["DOCKER_TEST_LOG"], "a") as log:
    log.write(json.dumps({"args": args, "env": {k: os.environ.get(k) for k in (
        "FLEET_CONFIG", "MPC_SCENARIO", "MPC_SETTINGS", "RUN_ID")}}) + "\\n")
if args[0] == "inspect":
    print('"sha256:test"')
elif "ps" in args and "-q" in args and "--status" not in args:
    print("test-container")
if args[-2:] == ["mission", "stop"] and os.environ.get("FAIL_MISSION_STOP"):
    sys.exit(1)
''')
    docker.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}", "DOCKER_TEST_LOG": str(log)}
    return log, env


def invoke(args, env):
    return subprocess.run([str(ROOT / "scripts/dev"), *args], env=env,
                          capture_output=True, text=True, timeout=10)


def calls(log):
    return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []


@pytest.mark.parametrize("args", [["sim"], ["flight", "takeoff"], ["flight", "position"],
                                  ["mpc", "--profile", "control"], ["mpc", "--resume", "old"]])
def test_legacy_commands_are_rejected_before_docker(fake_docker, args):
    log, env = fake_docker
    assert invoke(args, env).returncode == 2
    assert calls(log) == []


def test_mpc_passes_custom_configuration_to_compose(fake_docker):
    log, env = fake_docker
    result = invoke(["mpc", "--no-wait", "--fleet", "/workspace/config/custom-fleet.yaml",
                     "--scenario", "/workspace/custom-scenario.yaml", "--settings",
                     "/workspace/custom-settings.yaml"], env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        start = next(e for e in entries if "up" in e["args"])
        assert start["env"]["FLEET_CONFIG"] == "/workspace/config/custom-fleet.yaml"
        assert start["env"]["MPC_SCENARIO"] == "/workspace/custom-scenario.yaml"
        assert start["env"]["MPC_SETTINGS"] == "/workspace/custom-settings.yaml"
    finally:
        for entry in entries:
            if entry["env"]["RUN_ID"]:
                (ROOT / "runs" / (entry["env"]["RUN_ID"] + ".image-id.json")).unlink(missing_ok=True)


def test_stop_lands_mission_before_stopping_container(fake_docker):
    log, env = fake_docker
    assert invoke(["stop"], env).returncode == 0
    commands = [e["args"] for e in calls(log)]
    assert len(commands) == 2
    assert commands[0][-4:] == ["run", "solo_mpc", "mission", "stop"]
    assert commands[1][-2:] == ["stop", "sim"]


def test_failed_landing_keeps_container_running(fake_docker):
    log, env = fake_docker
    env["FAIL_MISSION_STOP"] = "1"
    assert invoke(["stop"], env).returncode == 1
    assert len(calls(log)) == 1


def test_force_stop_only_stops_processes(fake_docker):
    log, env = fake_docker
    assert invoke(["stop", "--force"], env).returncode == 0
    commands = [e["args"] for e in calls(log)]
    assert len(commands) == 1
    assert commands[0][-2:] == ["stop", "sim"]
