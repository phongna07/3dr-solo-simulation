import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('wait_ready', Path(__file__).parents[1] / 'scripts/wait_ready.py')
wait_ready = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wait_ready)


def test_resume_does_not_accept_health_from_before_container_restart():
    data = {'ready': True, 'updated_wall_unix': 100}
    assert not wait_ready.health_is_ready(data, now=101, container_started=100.5)
    data['updated_wall_unix'] = 101
    assert wait_ready.health_is_ready(data, now=102, container_started=100.5)
    assert not wait_ready.health_is_ready(data, now=112, container_started=100.5)


def test_stopped_container_with_exit_zero_reports_logs_immediately(monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[1] == 'inspect':
            return SimpleNamespace(returncode=0, stdout=json.dumps({'Status': 'exited', 'ExitCode': 0}))
        return SimpleNamespace(stdout='RuntimeError: plugin parameters rejected: invalid allocator')

    monkeypatch.setattr(wait_ready.subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='invalid allocator'):
        wait_ready.inspect_container('sim-container')
    assert len(calls) == 2


def test_created_container_cannot_accept_previous_ready_health(monkeypatch):
    monkeypatch.setattr(wait_ready.subprocess, 'run', lambda *args, **kwargs:
                        SimpleNamespace(returncode=0, stdout=json.dumps({'Status': 'created'})))
    started = wait_ready.inspect_container('sim-container')
    assert not wait_ready.health_is_ready({'ready': True, 'updated_wall_unix': 100}, 101, started)
