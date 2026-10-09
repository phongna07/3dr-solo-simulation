import json

import pytest

from solo_bringup.configuration import load_config
from solo_bringup.runtime import create_run
from test_configuration_world import ROOT


def test_state_isolation_and_explicit_resume(tmp_path, monkeypatch):
    monkeypatch.setenv('RUNS_ROOT', str(tmp_path))
    config = load_config(ROOT / 'config/fleet.yaml')
    first = create_run(config, 'simulation', 'control', 'first')
    state = first / 'sitl/0/eeprom.bin'
    state.parent.mkdir(parents=True)
    state.write_bytes(b'persisted parameters')
    second = create_run(config, 'simulation', 'control', 'second')
    assert (tmp_path / 'latest').resolve() == second
    assert not (second / 'sitl/0/eeprom.bin').exists()
    with pytest.raises(ValueError):
        create_run(config, 'simulation', 'control', 'first')
    (first / 'health.json').write_text('{}')
    resumed = create_run(config, 'simulation', 'control', 'first', resume=True)
    assert resumed == first
    assert state.read_bytes() == b'persisted parameters'
    assert not (first / 'health.json').exists()
    assert json.loads((first / 'manifest.json').read_text())['mode'] == 'simulation'
    with pytest.raises(ValueError):
        create_run(config, 'simulation', 'reference', 'first', resume=True)


@pytest.mark.parametrize('run_id', ['latest', 'hardware', '../outside', '/absolute'])
def test_reject_reserved_or_escaping_run_ids(tmp_path, monkeypatch, run_id):
    monkeypatch.setenv('RUNS_ROOT', str(tmp_path))
    with pytest.raises(ValueError):
        create_run({}, 'simulation', run_id=run_id)
