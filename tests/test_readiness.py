"""Validate diagnostic checks against the existing live readiness predicate."""

import json
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from solo_bringup import readiness


def test_readiness_explains_telemetry_pose_and_fleet_parameter_waits(tmp_path, monkeypatch):
    class Plugins:
        configured = False

        def __init__(self, *args):
            pass

        def tick(self):
            return self.configured

        def status(self):
            return {
                '/alpha/mavros/local_position': {'ready': True, 'phase': 'ready'},
                '/bravo/mavros/local_position': {'ready': self.configured, 'phase': 'applying parameters'},
            }

    monkeypatch.setattr(readiness, 'PluginParameters', Plugins)
    monkeypatch.setattr(readiness.time, 'monotonic', lambda: 100)
    rclpy.init()
    node = readiness.Readiness({'uavs': [{'name': 'alpha'}, {'name': 'bravo'}]},
                               tmp_path, timeout=600, max_pose_age=10)
    try:
        for record in node.records.values():
            record.update(state=SimpleNamespace(connected=True, armed=False, mode='STABILIZE'),
                          state_at=100, client=SimpleNamespace(service_is_ready=lambda: False))
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        checks = health['uavs']['alpha']['checks']
        assert checks['FCU']['ready']
        assert checks['MAVROS parameters']['ready']
        assert checks['telemetry']['detail'] == 'waiting for MAVROS set_message_interval service'
        assert checks['local pose']['detail'].startswith('waiting for first local pose')
        assert not health['ready']
        for record in node.records.values():
            record.update(intervals={30, 32}, pose_count=3, pose_at=100)
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert all(check['ready'] for check in health['uavs']['alpha']['checks'].values())
        assert not health['uavs']['alpha']['ready']  # The other UAV's parameters still gate the fleet.
        node.plugin_parameters.configured = True
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert health['ready']
        assert (tmp_path / 'ready.json').exists()
        node.records['alpha']['pose_at'] = 80
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert not health['ready']
        assert health['uavs']['alpha']['checks']['local pose']['detail'] == 'local pose stream stale'
    finally:
        node.destroy_node()
        rclpy.shutdown()
