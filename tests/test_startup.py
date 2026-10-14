"""Startup status must reflect lifecycle events without controlling readiness."""

import json
import os
import sys

import pytest

from solo_bringup.startup import ComponentProgress, StartupProgress


def test_startup_inventory_and_component_writes_are_independent(tmp_path):
    progress = StartupProgress(tmp_path)
    progress.stage('Generating world')
    progress.register('controller', initializes=True)
    assert json.loads(progress.path.read_text())['processes']['controller']['state'] == 'pending'
    progress.running('controller', os.getpid())
    component = ComponentProgress(tmp_path, 'controller')
    component.update('initializing', 'Constructing controller workers')
    component.update('initialized', 'Waiting for mission inputs')
    inventory = json.loads(progress.path.read_text())
    assert inventory['processes']['controller']['state'] == 'running'
    assert inventory['phase'] == 'Processes launched'
    assert json.loads(component.path.read_text())['state'] == 'initialized'
    progress.exited('controller', 7)
    assert json.loads(progress.path.read_text())['processes']['controller']['exit_code'] == 7
    assert not list(tmp_path.rglob('*.tmp'))


def test_launch_events_record_real_process_start_and_exit(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess
    from solo_bringup.launching import observed

    progress = StartupProgress(tmp_path)
    action = ExecuteProcess(cmd=[sys.executable, '-c', 'raise SystemExit(7)'])
    service = LaunchService()
    service.include_launch_description(LaunchDescription(observed(action, progress, 'test-process')))
    service.run()
    process = json.loads(progress.path.read_text())['processes']['test-process']
    assert process['pid'] > 0
    assert process['state'] == 'exited'
    assert process['exit_code'] == 7
