# Development

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Configuration](CONFIGURATION.md) · [Deployment](DEPLOYMENT.md)

Read [AGENTS.md](../AGENTS.md) before editing. Keep numerical code independent
of ROS and preserve measured-state control and independent flight supervision.

## Architecture

`scripts/dev` starts Docker Compose; ROS launch owns Gazebo, one SITL/MAVROS
pair per UAV, readiness, bridges, frame/perception/target nodes, the controller,
independent setpoint publisher, and mission supervisor. The GUI is a separate
client. Each persistent UAV worker owns its map, planner, corridor, solver,
committed path, warm start, and stopping plan.

| Location | Responsibility |
|---|---|
| `external/motion_planning_MPC/` | Colleague-owned, unchanged solver and predictive dynamics |
| `solo_mpc/core/upstream.py` | Private import of upstream solver/dynamics and source provenance |
| `solo_mpc/core/mpc_problem.py` | Local solver compatibility adjustments |
| `core/controller.py`, `coordination.py`, `tracking.py` | Measured-state planning, formation, estimation, retained stopping plans |
| `core/planner_grid.py`, `occupancy_grid.py`, `mapping.py` | Local JPS/Dijkstra, occupancy primitives, explicit sensor rays |
| `core/parallel_controller.py`, `cycles.py` | Persistent workers, frozen fleet predictions, deadlines, asynchronous cycles |
| `controller_node.py`, `frame_node.py`, `perception_node.py` | ROS input/output, alignment, marker detection |
| `setpoint_node.py` | Trajectory sampling, altitude feedback, command expiry, proximity guard |
| `mission_node.py`, `mission.py`, `evaluate.py` | Supervision, request CLI, measured-flight evaluation |
| `solo_bringup/` | Fleet configuration, generated runtime files, MAVROS bootstrap, readiness |
| `solo_mpc/gazebo/` | Native world-stabilized camera and coverage plugins |

`TeamController.step(states, scans, stamp, seers)` has no ROS imports and never
advances a simulated replacement for the measured vehicle. Workers freeze a
common prediction snapshot before solving. Spawn avoids inheriting DDS/native
solver state. Late results are rejected; timed-out workers stop until controller
restart. Independent publishing and supervision remain able to expire references
and request LAND.

## Upstream submodule

Initialize before building, or clone the parent with `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

Do not edit the submodule or the colleague's sibling checkout. To adopt a fetched
upstream commit, tag, or ref:

```bash
git -C external/motion_planning_MPC fetch origin
./scripts/pin-mpc-upstream COMMIT_OR_REF
```

The helper refuses a dirty submodule, checks out a detached revision, and updates
`ros_ws/src/solo_mpc/UPSTREAM.json` with its commit and solver/dynamics hashes.
It does not fetch, stage, or modify upstream files. Use `HEAD` to refresh metadata
for an already-selected revision. Inspect the upstream diff and validate a
mission, then include both the gitlink and metadata in your parent commit:

```bash
git add external/motion_planning_MPC ros_ws/src/solo_mpc/UPSTREAM.json
```

Only upstream `mpc_problem.py` and `dynamics.py` execute directly. Planning,
formation, target estimation, and configuration remain local ports; changes in
upstream `robot_jps.py`, planners, or `config.py` require deliberate integration.
A remote push or parent `git pull` does not select a newer upstream revision.

The loader binds upstream's flat `import dynamics` privately, without adding
upstream to global import paths or importing its simulator/configuration. It
executes unchanged source bytes, ignoring upstream's tracked bytecode, and writes
nothing into the submodule. Installed packages bundle the two sources and license
under `share/solo_mpc/upstream`; release runtime needs no Git checkout or network.

The adapter subclasses upstream `MPCProblem`. It retains inactive-neighbor
numerical conditioning and optional measured-overspeed recovery (disabled by
default), preserving the objective, active constraints, stopping branch, and
upstream solve routine. It checks constraint structure and fails explicitly if
an upstream change invalidates the adjustments. Optional SQP/IPOPT fallback also
uses the adapter. Do not restore copied solver/dynamics implementations.

## Edit and rebuild

Development mounts the checkout at `/workspace` and imports live Python sources.
Restart the stack for existing Python, launch, configuration, or upstream solver
edits. Rebuild for dependencies, package metadata, new executables/interfaces,
renamed modules, or native camera plugins:

```bash
./scripts/dev build sim
git diff --check
```

Dependency changes belong in Docker/locks, not manual container installs or the
upstream requirements file. Release images require rebuilding for any source
change. An experimental `colcon build --symlink-install` inside a shell affects
only an explicitly sourced overlay; it does not update the launch entrypoint.
Preserve sourcing order: ROS → pinned MAVROS → project overlay.

## Frames and clocks

MAVROS local origins are per UAV; MPC uses common `world` ENU. While disarmed,
the frame adapter calibrates translation against Gazebo truth using configured
yaw, then freezes alignment. Subsequent state comes from MAVROS. Positions need
rotation plus translation; velocity/acceleration need rotation only. MAVROS
converts local ENU setpoints to FCU NED.

MPC, sensors, target, frames, and setpoints use Gazebo `/clock`; MAVROS uses
wall/FCU time. Measured state is stamped at simulation receipt time and sensors
project against nearby pose history. This retains latency error. Trajectories
keep their original source epoch for elapsed-time sampling and expiry. Without
`/clock`, simulation-time node timers stop. See [timing settings](CONFIGURATION.md#timing-and-expiry).

The independent publisher imports no optimizer. With fresh state, expired
references produce zero horizontal velocity with altitude feedback. Stale pose
or FCU status removes altitude feedback; missing connection, armed/GUIDED state,
pose, or alignment can prevent output. A retained stop plan is used only while
measured state agrees with it. Do not weaken these checks to pass a mission.

## ROS interfaces

`uavN` denotes a configured vehicle; sensor topics use sensor-data QoS.

| Topic/service | Role |
|---|---|
| `/uavN/mavros/local_position/pose`, `/velocity_local` under the same prefix | FCU local pose/velocity |
| `/uavN/mpc/pose`, `/uavN/mpc/velocity` | World-aligned measured state |
| `/uavN/scan`, `/uavN/camera/image_raw`, `/uavN/camera/camera_info` | LiDAR and calibrated images |
| `/uavN/target/observation`, `/target/ground_truth` | Selected target input |
| `/uavN/mpc/trajectory` | Timestamped XY position/velocity/acceleration horizon |
| `/uavN/mpc/reference`, `/uavN/mpc/prediction` | Requested reference and predicted path |
| `/uavN/mavros/setpoint_raw/local` | Guarded FCU setpoint |
| `/mpc/status`, `/mpc/setpoint_status`, `/mpc/occupancy_grid` | Controller/publisher diagnostics and visualization map |
| `/mission/start`, `/mission/stop`, `/mission/reset`, `/mission/status` | Persistent mission requests |
| `/mpc/enable`, `/target/run`, `/target/reset` | Publishing and target ownership |
| `/solo/readiness`, `/clock` | Health and simulation clock |

### MAVROS bootstrap

Pinned MAVROS plugin nodes ignore global CLI parameter files. The bootstrap
uses parameter services, writes only changed values, and must finish before
readiness. The pinned `setpoint_attitude.use_quaternion` callback has a lifetime
bug; leave its correct default `false` untouched. Enabling quaternion control
requires fixing/rebuilding MAVROS and validating that change. Simulation disables
MAVLink timesync/SYSTEM_TIME and the FCU parameter plugin because SITL loads its
parameters and its clock differs from wall time.

## Verification and optimizer replay

Do not write automated tests. Run a representative mission, inspect measured
telemetry, logs and evaluation, and stop normally; see [Simulation](SIMULATION.md).
For a solver change, ground-truth target mode isolates optimization from camera
detection; add recording when visual behavior is relevant. Preserve the specific
run directory and report limits as well as successful outcomes.

`metadata/controller_sources.jsonl` and optimizer captures identify actual loaded
source hashes, the pinned commit, and `matches_pinned_sources`. Each UAV retains
up to `diagnostic_capture_cycles` complete calls under `optimizer-inputs/`.
Pending records survive worker termination; successful records include nominal
and backup plans/warm starts. Copy a record before the ring overwrites it.

Inside the simulation container:

```bash
PYTHONPATH=/workspace/ros_ws/src/solo_mpc OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1   python3 -m solo_mpc.replay_optimizer   /workspace/runs/RUN_ID/optimizer-inputs/uav1/solve-0001.json
```

Replay uses recorded budgets. `--iteration-only` removes the CPU cap for a
separate numerical diagnostic; it does not establish flight or timing success.
`SOLO_MPC_UPSTREAM_PATH` can explicitly select alternative upstream sources for
diagnostics; hash mismatches remain visible in provenance.

### Recorded verification

Cleanup run `20261029T044343Z-8504` rebuilt the image and repeated a headless
30-wall-second ground-truth-target mission after removing unused code and the
standalone hardware workspace. All UAVs landed/disarmed; 301 of 303 airborne
cycles were fleet-ready, with two CPU-cap failures. Minimum sampled body clearance
was 0.778 m and separation 1.818 m. Installed imports and live source hashes
confirmed the renamed occupancy grid, removed legacy modules, and unchanged
upstream pin. Evidence is in that run's `reports/cleanup.json`, `evaluation.json`,
mission status, telemetry, and command logs. Documentation links/import references
and `git diff --check` also passed.

Headless run `20261029T042434Z-29175` used upstream commit `3f71d4c`, measured
FCU state/LiDAR, and ground-truth target input for 30 wall seconds of tracking.
All three UAVs landed/disarmed. Of 306 airborne cycles, 304 were fleet-ready;
UAV 1 hit the unchanged CPU cap twice. Minimum sampled body clearance was
0.816 m; separation was 1.767 m. Fleet compute median/p95/max was
100.2/139.7/220.7 ms, so the 100 ms target was not consistently achieved.

The run's `reports/mpc-migration.json`, `evaluation.json`, source metadata,
optimizer inputs, and telemetry retain the evidence. An archived optimizer call
also replayed with zero state-trajectory difference from its pre-migration result.
Camera detections, a complete route, and hardware flight were outside that check.
Historical experiment details remain in their saved run directories and Git history.
