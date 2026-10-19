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


@pytest.mark.parametrize("count", [3, 5])
def test_workers_match_serial_with_shared_neighbors_and_persistent_maps(cfg, count):
    names = ["alpha", "bravo", "charlie", "delta", "echo"][:count]
    cfg["GRID_CELLS"] = [[0, 1], [0, -1], [-1, 0], [1, 0]]
    serial = TeamController(names, cfg)
    assert all(s.n_others == count - 1 for s in serial.solvers.values())
    with ParallelTeamController(names, cfg, timeout=3.0) as parallel:
        pids = {w.pid for w in parallel.workers.values()}
        assert len(pids) == count and os.getpid() not in pids
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


def test_big1000_workers_solve_measured_starts_and_shut_down():
    """One stationary offline cycle at field scale, with analytic obstacle scans."""
    scenario, settings, cfg = load_settings(PKG / "scenarios/big1000_5uav.yaml",
                                           PKG / "config/controller_big1000.yaml")
    cfg["IPOPT_OPTIONS"] = {**cfg["IPOPT_OPTIONS"], "ipopt.max_cpu_time": 1.0}
    names = [f"uav{i + 1}" for i in range(5)]
    states, scans = {}, {}
    angles = -np.pi + np.arange(180) * 2*np.pi/180
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    for n, point in zip(names, scenario["starts"]):
        state = VehicleState(np.asarray(point), np.zeros(3), np.zeros(3),
                             np.array([0., 0., 0., 1.]), 0.)
        states[n] = state
        ranges = np.full(180, cfg["SENSING_RADIUS"])
        for x, y, width, height in scenario["obstacles"]["rects"]:
            with np.errstate(divide="ignore", invalid="ignore"):
                near = (np.array([x, y]) - state.position[:2]) / directions
                far = (np.array([x + width, y + height]) - state.position[:2]) / directions
            entry = np.minimum(near, far).max(axis=1)
            exit = np.maximum(near, far).min(axis=1)
            ranges = np.minimum(ranges, np.where((entry >= 0) & (exit >= entry), entry, np.inf))
        for x, y, radius in scenario["obstacles"]["circles"]:
            delta = np.array([x, y]) - state.position[:2]
            projected = directions @ delta
            discriminant = projected**2 - (delta @ delta - radius**2)
            distance = projected - np.sqrt(np.maximum(discriminant, 0))
            ranges = np.minimum(ranges, np.where((discriminant >= 0) & (distance > 0), distance, np.inf))
        scans[n] = project_scan(ranges, -np.pi, 2*np.pi/180, cfg["LIDAR_RANGE_MIN"],
                                cfg["SENSING_RADIUS"], state.position, state.quaternion, 0.)
    with ParallelTeamController(names, cfg, timeout=10., startup_timeout=60.) as team:
        assert len({w.pid for w in team.workers.values()}) == 5
        assert team.solvers == {}
        team.track.observe(scenario["target"]["waypoints"][0], 0., variance=1.)
        results = team.step(states, scans, 0., {"uav4"})
        assert team.leader == "uav4" and len(team.assignment) == 4
        assert all(result["ok"] for result in results.values()), results
        for n, result in results.items():
            assert result["prediction"].shape == (21, 6)
            assert np.isfinite(result["prediction"]).all()
            assert team.planners[n].gmap.blocked.shape == (501, 501)
            np.testing.assert_array_equal(states[n].position, scenario["starts"][names.index(n)])
    assert all(not w.process.is_alive() for w in team.workers.values())
