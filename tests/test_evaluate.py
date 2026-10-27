"""Evaluate saved flight data from the organized run directories."""

import json
import sys

import yaml

from solo_mpc.evaluate import main


def test_evaluation_reads_snapshots_and_telemetry_and_saves_report(tmp_path, monkeypatch, capsys):
    for directory in ('config', 'telemetry'):
        (tmp_path / directory).mkdir()
    (tmp_path / 'config/scenario.yaml').write_text(yaml.safe_dump({
        'obstacles': {'rects': [], 'circles': [[10, 0, 1]]},
    }))
    (tmp_path / 'config/mpc-config.json').write_text(json.dumps({'ROBOT_RADIUS': 0.5}))
    records = [{
        'stamp': stamp, 'wall': stamp * 2, 'compute_ms': 50,
        'target_mode': 'ground_truth', 'reason': 'ready', 'target_estimate': [3, 0],
        'vehicles': {'alpha': {'position': [0, 0, 3]}},
    } for stamp in (1, 2)]
    truth = [{
        'stamp': stamp, 'target': [3, 0],
        'vehicles': {'alpha': {'position': [1, 0, 3]}},
    } for stamp in (1, 2)]
    for filename, rows in (('mpc.jsonl', records), ('truth.jsonl', truth)):
        (tmp_path / 'telemetry' / filename).write_text(
            ''.join(json.dumps(row) + '\n' for row in rows))
    monkeypatch.setattr(sys, 'argv', ['evaluate', '--run-dir', str(tmp_path)])

    main()

    report = json.loads((tmp_path / 'reports/evaluation.json').read_text())
    assert report == json.loads(capsys.readouterr().out)
    assert report['samples'] == 2
    assert report['geometry_source'] == 'gazebo_ground_truth'
    assert report['minimum_obstacle_body_clearance_m'] == 7.5
    assert report['final_uav_xy'] == {'alpha': [1, 0]}
    assert report['simulated_seconds_per_wall_second'] == 0.5
    assert all(entry.is_dir() for entry in tmp_path.iterdir())
