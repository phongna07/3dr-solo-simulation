"""Real spawned workers: numerical parity, concurrency and bounded failures."""

import os
from pathlib import Path
import signal
import time

import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("_pydecomp")
from solo_mpc.core.controller import TeamController, VehicleState
from solo_mpc.core.parallel_controller import ParallelTeamController
from solo_mpc.core.sensors import project_scan
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"


def inputs(names, stamp):
    states = {
        n: VehicleState(np.array([8., 4. + 1.2 * i, 3.]), np.zeros(3),
                        np.zeros(3), np.array([0., 0., 0., 1.]), stamp)
        for i, n in enumerate(names)
    }
    scans = {}
    for n, state in states.items():
        ranges = np.full(180, np.inf)
        ranges[90:94] = 2.5  # real occupied cells as well as clearing rays
        scans[n] = project_scan(ranges, -np.pi, 2*np.pi/180, .1, 3.,
                                state.position, state.quaternion, stamp)
    return states, scans


@pytest.fixture
def cfg():
    _, _, config = load_settings(PKG / "scenarios/small50_3uav.yaml",
                                  PKG / "config/controller.yaml")
    # Numerical equivalence, not a deadline benchmark on a shared CI machine.
    config["IPOPT_OPTIONS"] = {**config["IPOPT_OPTIONS"], "ipopt.max_cpu_time": 1.0}
    return config


def test_workers_match_serial_with_shared_neighbors_and_persistent_maps(cfg):
    names = ["alpha", "bravo", "charlie"]
    serial = TeamController(names, cfg)
    with ParallelTeamController(names, cfg, timeout=3.0) as parallel:
        pids = {w.pid for w in parallel.workers.values()}
        assert len(pids) == 3 and os.getpid() not in pids
        # The coordinator owns no native solvers or local occupancy integrators.
        assert parallel.solvers == {}
        for k in range(4):
            stamp = k * .1
            states, scans = inputs(names, stamp)
            positions = {n: s.position.copy() for n, s in states.items()}
            for team in (serial, parallel):
                team.track.observe([9. + .02*k, 4.], stamp)
            expected = serial.step(states, scans, stamp, {names[0]})
            actual = parallel.step(states, scans, stamp, {names[0]})
            assert parallel.leader == serial.leader
            assert parallel.assignment == serial.assignment
            for n in names:
                assert expected[n]["ok"] and actual[n]["ok"]
                np.testing.assert_allclose(actual[n]["prediction"], expected[n]["prediction"],
                                           atol=1e-7, rtol=1e-7)
                np.testing.assert_allclose(actual[n]["acceleration"], expected[n]["acceleration"],
                                           atol=1e-7, rtol=1e-7)
                np.testing.assert_array_equal(states[n].position, positions[n])
                serial.planners[n].gmap._ensure()
                np.testing.assert_array_equal(parallel.planners[n].gmap.blocked,
                                               serial.planners[n].gmap.blocked)
                assert actual[n]["worker_pid"] in pids
                assert actual[n]["cycle"] == k + 1
                assert actual[n]["mapping_ms"] >= 0
            # Each worker receives the same snapshot. Solves really overlap;
            # creating three processes alone would not establish concurrency.
            if k:
                assert max(r["solve_started_wall"] for r in actual.values()) < min(
                    r["solve_finished_wall"] for r in actual.values())
    assert all(not w.process.is_alive() for w in parallel.workers.values())
    parallel.close()  # idempotent shutdown


@pytest.mark.skipif(not hasattr(signal, "SIGSTOP"), reason="Requires POSIX process suspension")
def test_stalled_workers_have_one_deadline_and_never_use_stale_maps(cfg):
    names = ["alpha", "bravo", "charlie"]
    with ParallelTeamController(names, cfg, timeout=.2) as parallel:
        parallel.track.observe([9., 4.], 0.)
        for n in names[:2]:
            os.kill(parallel.workers[n].pid, signal.SIGSTOP)
        states, scans = inputs(names, 0.)
        started = time.perf_counter()
        result = parallel.step(states, scans, 0., {names[0]})
        assert time.perf_counter() - started < 1.0
        assert all(not r["ok"] for r in result.values())
        assert result["alpha"]["reason"] == "worker_timeout: map"
        assert result["bravo"]["reason"] == "worker_timeout: map"
        assert result["charlie"]["reason"] == "fleet_mapping_failed"
        # No queued late response may be reused in a later decision.
        assert all(p is None for p in parallel.predictions.values())
        assert all(not w.process.is_alive() for n, w in parallel.workers.items() if n != "charlie")
        assert all(not r["ok"] for r in parallel.step(states, scans, .1, {names[0]}).values())
    assert all(not w.process.is_alive() for w in parallel.workers.values())


def test_single_vehicle_worker_handles_empty_neighbor_set_and_disconnect(cfg):
    with ParallelTeamController(["solo"], cfg, timeout=3.) as parallel:
        parallel.track.observe([9., 4.], 0.)
        states, scans = inputs(["solo"], 0.)
        assert parallel.step(states, scans, 0., {"solo"})["solo"]["ok"]
        worker = parallel.workers["solo"]
        worker.process.terminate()
        worker.process.join(timeout=1.)
        result = parallel.step(states, scans, .1, {"solo"})["solo"]
        assert not result["ok"]
        assert "worker_disconnected" in result["reason"]


def test_big1000_five_workers_solve_with_four_neighbors_and_follower_slots():
    # Synthetic measured inputs exercise the real numerical workers without
    # starting Gazebo, SITL, MAVROS or integrating simulated vehicle motion.
    from solo_bringup.configuration import load_config

    from scipy.spatial.transform import Rotation

    scenario, settings, cfg = load_settings(PKG / 'scenarios/big1000_5uav.yaml',
                                     PKG / 'config/controller_big1000.yaml')
    names = [u['name'] for u in load_config(PKG.parents[2] / 'config/fleet_5uav.yaml')['uavs']]
    cfg['IPOPT_OPTIONS'] = {**cfg['IPOPT_OPTIONS'], 'ipopt.max_cpu_time': 1.0}
    serial = TeamController(names, cfg)
    assert all(solver.n_others == 4 for solver in serial.solvers.values())
    with ParallelTeamController(names, cfg, timeout=5.0, startup_timeout=60.0) as parallel:
        assert len({w.pid for w in parallel.workers.values()}) == 5
        for step in range(2):
            stamp = step * cfg['TIMESTEP']
            states = {
                n: VehicleState(np.array(start, float), np.zeros(3), np.zeros(3),
                                Rotation.from_euler('y', 5, degrees=True).as_quat(), stamp)
                for n, start in zip(names, scenario['starts'])
            }
            scans = {
                n: project_scan(np.full(180, np.inf), -np.pi, 2*np.pi/180,
                                .5, 50., s.position, s.quaternion, stamp,
                                vertical_bounds=(.3, settings['obstacle_height'] - cfg['ROBOT_RADIUS']))
                for n, s in states.items()
            }
            for team in (serial, parallel):
                team.track.observe([100., 100.], stamp, variance=1.0)
            expected = serial.step(states, scans, stamp, {'uav3'})
            actual = parallel.step(states, scans, stamp, {'uav3'})
            assert parallel.leader == serial.leader == 'uav3'
            assert parallel.assignment == serial.assignment
            assert set(parallel.assignment) == set(names) - {'uav3'}
            assert set(parallel.assignment.values()) == {0, 1, 2, 3}
            for n in names:
                assert expected[n]['ok'], expected[n]
                assert actual[n]['ok'], actual[n]
                assert actual[n]['prediction'].shape == (21, 6)
                np.testing.assert_allclose(actual[n]['prediction'], expected[n]['prediction'], atol=1e-6)
                assert np.max(np.linalg.norm(actual[n]['prediction'][:, 2:4], axis=1)) <= 10.0001
    assert all(not w.process.is_alive() for w in parallel.workers.values())
