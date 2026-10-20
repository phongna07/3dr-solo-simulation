# Controller parity audit and offline verification — 20 October 2026

The ROS port now restores the upstream coordination and MPC input assembly while
keeping measured-state control, physical sensing, process isolation, and command
expiry. Neither `core/mpc_problem.py` nor `core/dynamics.py` changed. Passing
identical-input checks establishes algorithmic parity at those boundaries; it
does not establish a successful Gazebo route or real-time performance.

Reference: `/home/phong/motion_planning_MPC`, inspected against its
`runs/scen4_n5_s0_20261019-150106/config.json` and current source definitions.
Failed ROS evidence: `/home/phong/3dr-solo/runs/latest-archive`.
No Gazebo server, SITL, or flight mission was launched for this work. Initial
working tree was clean; no unrelated changes were replaced.

## Accidental omissions and parameter drift restored

- **Configuration resolution:** scenario overrides precede controller overrides;
  dependent values are calculated afterwards unless explicitly pinned. Formation
  gap, exit hysteresis, switch margin, separation, repulsion distance, inflation,
  and standoff follow the upstream formulas. The large profile now uses a 12.5 m
  assignment switch margin, 3 m separation, 7.5 m repulsion distance, original
  `[N,W,S,E]` slot order, 0.3 minimum slot ratio, and 20 m path deviation trigger.
  `ipopt.acceptable_iter` is restored to 15. The small profile retains its
  explicitly specified physical speed/size/separation settings and 2 s slot
  lookahead; the large profile uses the upstream 5 s lookahead.
- **Target estimation/prediction:** the CV Kalman filter now uses the original
  initial covariance (velocity variance 100), process noise, and configured
  observation noise. At matching observation epochs its state and covariance
  match upstream. The MPC target horizon starts at the latest observation,
  with velocity capped at `2 * VMAX`. Planning uses travel time to the filtered
  target, the original velocity-confidence gate, and speed-dependent lead gain,
  rather than using the end of the MPC target horizon as the planning goal.
- **Leader selection:** shared sticky leader; retain it while relaxed visibility
  holds, otherwise choose minimum infinity distance among strict, then relaxed,
  seers, with fleet-order tie breaking. Upstream declares handoff counters but
  its active code deliberately uses sticky selection, so no proximity handoff
  or extra handoff delay was introduced.
- **Per-UAV modes:** a leader is TRACK immediately. Distant followers remain
  SEARCH and pursue the target even while another UAV is leader. A follower
  enters TRACK after three qualifying distance cycles and exits after ten
  distance violations. Entry is `VIEWING_RADIUS + FORMATION_GAP`; exit also
  accounts for its previous outer slot. Loss of every seer immediately returns
  the fleet to SEARCH, as upstream does.
- **Assignment:** one Hungarian assignment for all followers, including followers
  still in SEARCH, is computed when a TRACK follower needs slots. The previous
  permutation is retained unless improvement reaches `SWITCH_MARGIN`. Candidate
  count, optional open slots, and additional cells follow upstream semantics.
- **Contraction:** shared maps distinguish actual occupancy, inflation, and
  unknown cells. Slot rays stop at actual obstacles or map boundaries; a slot
  endpoint must be outside known inflation. Unknown space alone does not contract
  a slot. Leader predictions and terminal-velocity extrapolation are checked up
  to `SLOT_LOOKAHEAD_TIME`; contraction is immediate, expansion rate limited.
- **Moving references:** distant TRACK followers use a carrot one maximum-speed
  horizon along the path, not merely its next vertex. When the path is shorter,
  the whole leader prediction plus the contracted offset becomes the slot
  trajectory. SEARCH weight is per UAV; target standoff is disabled only for
  satellites with an active slot. Guide/standoff switches retain the MPC's
  original cost terms.
- **Paths/corridors/warm starts:** deviation-triggered replanning and reuse of a
  clear committed path after planning failure are restored. Temporary duplicate
  pose paths are allowed only before the planner failure threshold, without
  accidentally committing them as successful paths. The first new polytope
  supplies corridor cost independently of the retained hard stopping corridor,
  matching upstream. Solver face capacity grows in blocks of 16; nominal warm
  starts shift with repeated terminal rows. The numerical solver retains its
  own shifted backup warm start.

## Required ROS/physical adaptations retained

- Camera detections, never filtered predictions alone, establish the supplied
  visibility set. Strict/relaxed geometric checks qualify detections; they do
  not invent detections. Production profiles remain `target_mode: camera`.
  Target truth is confined to the existing explicitly selected diagnostic mode
  and isolated synthetic comparison tests. No target-route knowledge is supplied
  to the controller. The target generator's leg speeds are not MPC predictions.
- Observation stamps determine Kalman update intervals. Invalid frames, nonfinite
  positions, future/stale detections, and older filter updates are rejected.
  Between observations the port coasts for at most 1.5 s and decays predicted
  velocity; only genuine observations renew `observed_at`. The existing
  target-search timeout still suppresses control after prolonged loss. A shared
  fleet filter replaces upstream's identical per-UAV truth-fed filters; handling
  correlated/noisy multi-camera measurements remains an integration limitation.
- Explicit measured scan rays populate each persistent SensorGrid. Missing rays
  are not carved free; self/teammate masking, historical sensor pose, full attitude
  projection, height rejection, and minimum valid scan checks remain. The grid
  integrator differs from upstream's analytic/binned synthetic scans. Corridors
  use remembered occupied cell centers rather than raw current LiDAR endpoints.
  This can change the resulting path and polytope even for a similar scene.
- Production hard corridor margin adds `GRID_RESOLUTION / sqrt(2)` for cell
  quantization, plus the existing range noise and angular sampling allowance.
  In the large profile it is **2.182379719 m**, versus **0.768166156 m** in the
  reference snapshot. The large neighbor constraint range remains **55 m**
  versus upstream **45 m**. These are intentional physical geometry differences;
  offline upstream comparisons use the reference values on both sides.
- Measured FCU position/velocity/estimated acceleration are the optimizer state.
  No exact model integration overwrites measurements. World/ENU transformations
  and MAVROS/ArduCopter setpoint handling remain in the existing frame and
  independent publishing nodes. The small profile uses Iris physical limits;
  large-profile WPNAV speed/acceleration limits remain unchanged.
- The port trims a reused path by segment projection to the measured pose,
  retaining the next turn, rather than returning a potentially behind-the-UAV
  committed prefix. This existing physical-state adaptation is retained and
  tested; exact committed-path histories need not match the 2D simulation.
- A coherent fleet prediction snapshot is frozen before any solve. The reference
  defers state integration until all solves finish, but earlier solves can still
  replace predictions read by later solves. The port deliberately exchanges
  predictions between cycles, not within a cycle. Both implementations receive
  the same frozen neighbor snapshots in parity tests. Cached six-component plans
  are advanced by elapsed simulation steps, reanchored to measured position,
  and replaced by measured constant-velocity predictions when older than three
  timesteps or after a clock reversal. Keeping velocity permits slot lookahead.
- Verified stopping tails advance by elapsed simulation time, not a solve count.
  Replay requires measured position/velocity/acceleration agreement with the
  branch; it is rechecked against callbacks received during optimization before
  publication. Failed nominal warm starts are discarded. A missing/divergent
  certificate fails closed to publisher expiry/hold; the unverified model brake
  used by upstream is not presented as a safe physical trajectory. Fallback is
  always `nominal_ok: false`, so it cannot conceal failure from supervision.

## Runtime ownership and solver policy

Each UAV still has one persistent spawned process owning its grid, planner,
corridors, MPC, stopping plan, and warm starts. The coordinator owns shared
modes, leader, assignment, slot scales, filter, and prediction exchange. Workers
now return occupancy and unknown masks as well as inflation for contraction.
Mapping finishes fleet-wide before coordination. A single background coordinator
thread runs one frozen cycle; sensor callbacks continue on the ROS executor.
There is no queue of future cycles. A second concurrent `step` is rejected.

Both ROS execution choices retain those persistent per-UAV processes. `parallel`
dispatches solves together; `sequential` dispatches them in order against the
same snapshot and shared phase deadline. Direct `TeamController` remains useful
for offline numerical comparison. Worker startup/phase timeouts, wrong-cycle
response rejection, termination/kill escalation, disconnect handling, and joined
shutdown remain bounded. The independent publisher and persistent mission
supervisor remain separate processes; their expiry, failure timeout, pilot
ownership and landing rules were not relaxed.

The production profiles retain their **0.2 s / 0.5 s IPOPT CPU limits** and
**1 s / 2 s worker phase deadlines** (small/large). A CPU limit differs from a
wall deadline and neither guarantees 10 Hz. Completed commands keep the source
cycle stamp; simulation or wall expiry, a clock reversal, or loss of armed GUIDED
state rejects results instead of restamping them as new. Sensor freshness gates
remain in force.

Offline upstream comparisons use the saved reference's exact IPOPT options:
500 iterations, tolerance 1e-4, acceptable tolerance 1e-2, 15 acceptable iterations,
and **no explicit CPU-time cap**. This separates algorithmic differences from
machine-dependent deadlines. It is not a recommendation to increase production
timeouts. The production five-worker probe can still report nominal iteration
or CPU-budget failures with restored SEARCH behavior on dispersed starts;
classification and safe failure remain tested. Meeting runtime deadlines on the
simulation/flight machine is unresolved.

## Offline verification and evidence

`tests/test_upstream_parity.py` reads actual reference method/class definitions
without importing its simulator. The current source supplies mixed-case cost
weights omitted by the historical snapshot's `isupper()` filter; saved scenario
overrides and effective uppercase parameters are then applied. This is an
explicit configuration reconstruction for controlled tests, not an exact archive
replay. Source AST comparison verifies that the numerical MPC is unchanged;
dynamics bytes are identical.

The **20 direct upstream tests** cover three/five-UAV mode entry/exit, distant
followers, sticky and replacement leaders, assignment hysteresis, contraction
with occupied/inflated/unknown cells, future leader motion, moving slots, path
carrots, Kalman state/covariance, target horizons and lead pursuit, real JPS paths
on identical grids, complete normalized solver arguments, cold and shifted warm
starts, real nominal/backup solver outputs, and stopping fallback indexing.
Inputs use absolute tolerance **1e-10** and relative **1e-12** after the solver's
existing shape normalization (XY vs XYZ and flat vs column bounds). Nominal and
backup outputs use **atol=rtol=1e-7**. Mode/assignment decisions are exact;
filter/geometry probes use **1e-12**. Controlled distant SEARCH probes compare
failure outcomes/status too. These tolerances are local numerical checks, not
physical tracking tolerances.

Real spawned-worker comparisons run three and five UAVs for four cycles and
compare recorded assembled solver inputs, deterministic trajectories, map masks,
coherent neighbors, persistent PIDs, and overlapping parallel solve intervals.
Further checks cover suspended/disconnected workers, initialization failure,
wrong-cycle replies, nonoverlap, joined shutdown, callback responsiveness,
invalid observations, command expiry, measured fallback rejection, and complete
capture/replay including the backup warm start.

Host results: **276 passed, 4 skipped, 3 deselected**, using `.venv`, ROS Jazzy and
the existing `artifacts/mpc-sync-interface-install` overlay, single-threaded BLAS,
and ROS domain 73. Four skips require the simulation image/model environment.
The three deselected cases are the previously documented cross-process mission
service discovery/reply tests (two detached-supervisor cases and remote takeoff
cancellation); they were not rerun. Log:
`artifacts/mpc-parity-regression-20261020.log`. Replay CLI/settings and worker/ROS checks also passed (**62 focused tests**),
recorded in `artifacts/mpc-parity-final-focused-20261020.log`. Direct comparison
output is in `artifacts/mpc-parity-upstream-20261020.log`. No optimizer performance or flight
success claim is based on these counts.

An initial planner probe inadvertently regenerated Numba cache files in the
reference `__pycache__`. The affected entries were removed from the reference
and retained under `artifacts/parity-reference-cache-sideeffects`; source and
saved run/log files were unchanged. The final harness disables Numba disk caches
and fingerprints reference sources, caches and the supplied run before/after its
tests. This correction also avoids invalid dynamic-module cache reuse.

## Future replay capture

The failed archive contains 518 recorded `Solve_Succeeded` statuses and 62
`Maximum_CpuTime_Exceeded` statuses; all its recorded unsuccessful solver statuses
are CPU-limit failures. It lacks complete optimizer inputs, corridor histories,
and warm starts. Neither those failed solves nor the original mission were
exactly replayed.

With a run directory, each worker now atomically captures the last
`diagnostic_capture_cycles` optimizer calls (default **128** per UAV; **0** disables
capture) under `optimizer-inputs/<uav>/solve-NNNN.json`. A **pending** record is
written before entering IPOPT, so worker termination still leaves its inputs.
Records contain effective configuration, dimensions, all solver arguments,
nominal initial guesses, internal backup initial guesses, source hashes,
simulation/wall timestamps, outcome/statistics, and solved nominal/backup arrays.
They enable optimizer replay, not full sensor/planner replay; map histories,
raw ROS messages, and camera images are not reconstructed by this facility.
The ring bounds disk use; copy a run before older calls are overwritten.

```bash
PYTHONPATH=ros_ws/src/solo_mpc OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python -m solo_mpc.replay_optimizer \
  runs/NEW_RUN/optimizer-inputs/uav1/solve-0001.json
```

The default uses recorded budgets. `--iteration-only` explicitly removes the
CPU cap for an isolated diagnostic; it changes the budget and cannot be called
an exact replay of a CPU-limited outcome. CPU timing, dependency versions and
platform differences can change an outcome even with complete inputs.

## Commands for later validation on the simulation machine

These commands are provided for later use; no simulation or mission command
below was executed for this change.

First run the read-only offline comparison (it requires the reference checkout
and matching saved run on that machine):

```bash
.venv/bin/python scripts/verify_mpc_parity.py \
  --reference /home/phong/motion_planning_MPC \
  --reference-run /home/phong/motion_planning_MPC/runs/scen4_n5_s0_20261019-150106
```

For the existing host ROS test environment:

```bash
source /opt/ros/jazzy/setup.bash
source artifacts/mpc-sync-interface-install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_DOMAIN_ID=73 \
  .venv/bin/python -m pytest tests -q \
  -k 'not test_start_cli_exit_keeps_supervision_and_automatic_landing and not test_cli_cancellation_interrupts_remote_takeoff_with_paused_sim_clock'
```

Then build and launch the disarmed five-UAV camera profile separately:

```bash
./scripts/dev build sim
./scripts/dev mpc \
  --fleet /workspace/config/fleet_5uav.yaml \
  --scenario /workspace/ros_ws/src/solo_mpc/scenarios/big1000_5uav.yaml \
  --settings /workspace/ros_ws/src/solo_mpc/config/controller_big1000.yaml
```

After readiness, use `./scripts/dev mission run --seconds 20` for the short
supervised check; the existing `./scripts/dev mission run` performs the full
route. For three UAVs use the default `./scripts/dev mpc` profile in a separate
run. Preserve source/config/image identity, `mpc.jsonl`, optimizer captures,
sensor ages, seer/mode/leader histories, measured tracking errors, publisher
jitter/expiry, fallback acceptance, worker failures and landing result. Confirm
that distant followers stay SEARCH until entry hysteresis, leader loss requires
real camera reacquisition, and failed/late solves never keep the supervisor
healthy. Complete route quality, camera reliability, stopping tolerances under
FCU model mismatch, continuous clearance, and achieved cycle rates remain to be
validated. No cost-function tuning is justified by parity checks alone.

## Changed files

- Controller/input assembly: `solo_mpc/core/controller.py`, new
  `core/coordination.py`, `core/tracking.py`.
- Runtime adapter/ownership: `solo_mpc/controller_node.py`,
  `core/parallel_controller.py`, new `core/cycles.py`.
- Replay: new `core/diagnostics.py`, `solo_mpc/replay_optimizer.py`, and
  `scripts/verify_mpc_parity.py`.
- Effective settings: `solo_mpc/settings.py`, `config/controller.yaml`,
  `config/controller_big1000.yaml`, `scenarios/big1000_5uav.yaml`, and
  `solo_mpc/UPSTREAM.json`. Package-relative paths above are under
  `ros_ws/src/solo_mpc`.
- Verification: new `tests/upstream_oracle.py`, `tests/test_upstream_parity.py`,
  `tests/test_mpc_cycles.py`; updated `tests/test_parallel_mpc.py`,
  `tests/test_mpc_contingency.py`, `tests/test_mpc_ros.py`.
- Documentation: this report, `docs/CONFIGURATION.md`, `docs/DEVELOPMENT.md`,
  `docs/VALIDATION.md`.
