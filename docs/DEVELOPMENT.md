# Development

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Configuration](CONFIGURATION.md) · [Deployment](DEPLOYMENT.md)

Read [AGENTS.md](../AGENTS.md) before editing. Keep numerical code independent
of ROS and preserve measured-state control and independent flight supervision.

## Architecture

`scripts/dev` starts Docker Compose; `solo_mpc/launch/mpc.launch.py` owns Gazebo,
one native ROS flight process per UAV, readiness, bridges, frame/perception/target
nodes, the algorithm adapter, independent setpoint publisher, and mission
supervisor. The GUI is a separate
client. Each persistent UAV worker owns its map, planner, corridor, solver cache,
committed path, warm start, and stopping plan.

| Location | Responsibility |
|---|---|
| `external/motion_planning_MPC/` | Colleague-owned, unchanged solver and predictive dynamics |
| `solo_mpc/core/upstream.py` | Private import of upstream solver/dynamics and source provenance |
| `solo_mpc/core/mpc_problem.py` | Local solver compatibility adjustments |
| `core/algorithm.py`, `core/mpc_algorithm.py` | ROS-free `Algorithm`/`VehicleState` boundary and existing MPC integration |
| `core/controller.py`, `coordination.py`, `tracking.py` | Measured-state planning, formation, estimation, retained stopping plans |
| `core/planner_grid.py`, `occupancy_grid.py`, `mapping.py` | Local JPS/Dijkstra, occupancy primitives, explicit sensor rays |
| `core/parallel_controller.py`, `cycles.py` | Persistent workers, frozen fleet predictions, deadlines, asynchronous cycles |
| `controller_node.py`, `frame_node.py`, `perception_node.py` | ROS input/output, odometry twist rotation, marker detection |
| `setpoint_node.py` | Source-stamped trajectory sampling, position/altitude feedback, expiry and proximity guard |
| `flight_node.py` | Per-UAV ownership, flight service/state, world-to-body commands, takeoff and landing |
| `mission_node.py`, `mission.py`, `evaluate.py` | Supervision, request CLI, measured-flight evaluation |
| `solo_bringup/` | Fleet validation, run snapshots, readiness and flight clients |
| `solo_mpc/gazebo/` | Independent `SoloFlightSafety`, world-stabilized camera and coverage plugins |
| `solo_mpc/models/` | Vendored original Iris assets, inertias, rotor geometry and licenses |

The launch resolves fleet/scenario/controller inputs once for world generation,
bridges and ROS nodes. `solo_mpc/world.py` builds the full Iris fleet and sensor
world together; launch callbacks supervise process lifecycles. Every run is
fresh; old sensor/flight state
cannot be resumed. `core/visual_style.py` shares the camera framing and fleet
palette between the GUI, world and recorder.

Each vehicle uses native `MulticopterVelocityControl` → `Actuators` component →
four `MulticopterMotorModel` systems applying rotor force/torque dynamics to the
vendored original Iris model. No direct pose/velocity replacement or legacy
backend is used. ROS commands cross `SoloFlightSafety` before reaching the native
velocity controller; bridges never expose a bypass to its internal control input.

The small ROS-free `Algorithm` protocol exposes `VehicleState`, `track`,
`step(states, scans, stamp, seers, *, tracker=None)`, `occupancy_maps()`,
`diagnostics()`, and `close()`. `VehicleState` carries measured position,
velocity, acceleration, quaternion and source stamp. The factory setting is
`algorithm: mpc`; `core/mpc_algorithm.py` wraps the existing persistent MPC workers
without changing the colleague's solver/dynamics or importing ROS into the core.

`TeamController.step(states, scans, stamp, seers)` has no ROS imports and never
advances a simulated replacement for the measured vehicle. Workers freeze a
common prediction snapshot before solving. Spawn avoids inheriting DDS/native
solver state. Late results are rejected; timed-out workers stop until controller
restart. Independent publishing and supervision remain able to expire references
and request landing.

One controller owns at most one outstanding fleet future. A simulation-time
timer checks when the next `control_period` is due; it never polls completed
computations. The future's done callback only triggers a ROS guard condition.
The guard callback collects the fleet barrier's result on the ROS executor,
revalidates age, flight state and retained stopping plans, then publishes.
Timer, input and guard callbacks share the default mutually exclusive callback
group. A ready completion can start the next cycle immediately if its simulation
period has elapsed; an early completion waits for the scheduling timer.

Submission still deep-copies measured states, scans, seers and the tracker.
Every worker solves against the same frozen fleet predictions and guidance;
completion never feeds one UAV's new plan into another UAV's current solve.
References and trajectories retain the common input timestamp, regardless of
notification or delivery time. Independent setpoint sampling, expiry, ownership
and landing checks are unchanged. The collector consumes its context and future
before publishing, including exceptional futures, so stale/coalesced guard
notifications cannot process a cycle twice. Shutdown disables notification,
joins the bounded computation and discards its result before destroying the guard.

Completed-cycle records in `telemetry/mpc.jsonl` include monotonic wall-clock
`fleet_started_wall`, `fleet_completed_wall` and `fleet_collected_wall` in seconds.
Completion is stamped just before the fleet callable returns or raises;
collection is stamped when the executor takes the done future, before result
validation. `completion_to_collection_ms` measures that interval, including
future notification and executor dispatch. `compute_ms` remains total wall time
from input preparation through reference/trajectory publication; optional map
publication follows it. Idle/input-rejected records have no completion fields.
Compare complete airborne, owned TRACK records using identical resolved inputs
and recording settings; solver work and host load can vary between flights.

Per-UAV `solver_ms` directly brackets `MPCProblem.solve()` with a monotonic
wall timer; `solver_cpu_ms` measures process CPU over the same call. Neither
includes path planning, corridor construction, solver-cache selection or
diagnostic capture. `mapping_ms`, `path_ms`, `corridor_ms`, `solver_setup_ms`,
`input_ms` and `capture_ms` report those separate stages (capture includes
pre-solve journaling and post-solve serialization/queueing). `solver_setup_ms`
includes construction on a cache miss; native initialization deferred to the
first solve stays in `solver_ms`. `planning_and_solve_ms` remains the whole
worker solve phase, including those stages and result bookkeeping, excluding
the preceding map phase. Fleet `compute_ms` includes coordination, worker
barriers and delivery; concurrent per-UAV timings must not be summed to
derive it. These are wall milliseconds except for explicitly labelled CPU.

Each worker integrates its UAV's valid projected rays independently, once per
new scan timestamp, before formation checks and MPC use the latest accumulated
maps. No fleet-wide ray-count gate applies. Empty scans are a map no-op; scan
freshness and pose validation still gate control. `telemetry/mpc.jsonl` records
`scan_stamp`, `scan_usable_rays`, `scan_hits`, and `map_updated` per UAV so sparse
and empty scan cycles can be inspected alongside solver outcomes. Repeated scan
timestamps do not integrate the same evidence twice. See
[sensor geometry and projection limits](CONFIGURATION.md#sensors-and-target-input).

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

For the latest fetched `feature/compare` revision, use
`./scripts/pin-mpc-upstream origin/feature/compare`. The submodule stays detached
at that exact commit rather than following future branch updates automatically.

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
In particular, selecting `feature/compare` does not enable its upstream-only
comparison controllers in the ROS simulation.

The loader binds upstream's flat `import dynamics` privately, without adding
upstream to global import paths or importing its simulator/configuration. It
executes unchanged source bytes, ignoring upstream's tracked bytecode, and writes
nothing into the submodule. Installed packages bundle the two sources and license
under `share/solo_mpc/upstream`; release runtime needs no Git checkout or network.

The adapter subclasses upstream `MPCProblem`. It retains inactive-neighbor
numerical conditioning and optional measured-overspeed recovery (enabled by the
supplied controller), preserving the objective, other active constraints,
stopping branch, and upstream solve routine. It checks constraint structure and
fails explicitly if an upstream change invalidates the adjustments. Optional SQP/IPOPT fallback also
uses the adapter. Do not restore copied solver/dynamics implementations.

Each local UAV now owns a bounded `SolverCache` of unchanged upstream problems.
Selection uses exactly `max(1, hard face count, cost face count)`; neither
polytope is truncated or simplified, and both nominal and full backup horizons
remain enabled. Sizes in `CORRIDOR_CACHE_FACES` (default 4–8) and the configured
`CORRIDOR_MAX_FACES` (normally 16)
are constructed and solved with synthetic stationary inputs before worker
readiness, including any SQP/IPOPT fallback. Initialization failure prevents
readiness. That configured size is a preinitialized capacity, not a minimum for
every solve. Anticipated instances stay resident; two additional LRU slots hold
other exact counts (eight instances per UAV with the default configuration;
`scen1` anticipates 4–12 and 16, with twelve instances including the reserve).
An unexpected size is built inside the existing worker deadline; a miss can
still be slow or time out. Cache size does not limit actual corridor geometry.

The controller carries its current shifted nominal X/U guess across sizes;
those dimensions do not depend on face count. On every instance switch, backup
initial guesses and cached backup outputs are reset on the selected solver and
its fallback, preventing reuse of guesses from an earlier visit. Consecutive
calls to the same instance retain upstream backup warming. The separately
validated stopping plan is retained with its original simulation timestamp.
This changes only local solver ownership/selection: one fleet cycle still uses
one frozen state/prediction/guidance snapshot, completed trajectories retain its
source epoch, and the independent publisher retains expiry and application
checks. Smaller solves do not establish a 100 ms fleet deadline.

## Edit and rebuild

Development mounts the checkout at `/workspace` and imports live Python sources.
Restart the stack for existing Python, launch, configuration, or upstream solver
edits. Rebuild for dependencies, package metadata, new executables/interfaces,
renamed modules, vendored models, or native plugins:

```bash
./scripts/dev build sim
git diff --check
```

Dependency changes belong in Docker/locks, not manual container installs or the
upstream requirements file. Release images require rebuilding for any source
change. An experimental `colcon build --symlink-install` inside a shell affects
only an explicitly sourced overlay; it does not update the launch entrypoint.
Preserve sourcing order: ROS → project overlay.

## Frames and clocks

Gazebo `OdometryPublisher` supplies measured world ENU pose and body-frame twist
on `/uavN/ground_truth/odometry`. `frame_node` rotates the twist by the measured
orientation into common `world` ENU, retains the original source stamp for pose
and velocity, and builds timestamp-matched pose history for sensors. No per-UAV
origin calibration, receipt-time restamping or other navigation frame is needed.
Acceleration is estimated from measured world velocities, not predicted motion.

MPC, sensors, target, frames, flight supervision and setpoints use Gazebo
`/clock`. Trajectories retain their original source epoch for elapsed-time
sampling and expiry; never replace it with completion/publication time. Without
`/clock`, simulation-time node timers stop, but independent wall watchdogs still
apply. See [timing settings](CONFIGURATION.md#timing-and-expiry).

The default requested physics rate is 0.2× wall time. Gazebo retains the original
integration step; native vehicle control and rotor dynamics advance with physics.
Control and sensor rates, prediction horizons, and video sampling stay in
simulation seconds. Worker/CPU deadlines and wall freshness remain independent.
Evaluation reports both wall computation time and simulation input-to-result
delay; the former can exceed 100 ms while the latter meets a 0.1 s control period.

The independent publisher imports no optimizer. It samples source-stamped
`JointTrajectory` position/velocity/acceleration points, adds measured-position
and altitude feedback, and publishes world `TwistStamped` on
`/uavN/flight/velocity`. The trajectory retains its original input stamp and
receipt epoch; resampling cannot renew an expired plan. The velocity command is
stamped at the independent sampler's current simulation tick, so the flight
backend separately detects a dead sampler. Fresh state permits guarded hold output
on expiry; stale state, invalid ownership or flight mode prevents unsafe output.
A retained stop plan is used only while measured state agrees with it.

Each UAV's flight process exclusively publishes body `Twist` on `/uavN/cmd_vel`
and `Bool` on `/uavN/enable`. It rotates world requests into body axes, checks
source and receipt ages, and owns takeoff/land state transitions. `FlightCommand`
requires an owner token; `FlightState` reports connection, armed state, mode,
owner and landed status. Only settled ground and measured motor shutdown permit
disarm/release; low altitude or a disable request alone is insufficient.

The C++ `SoloFlightSafety` Gazebo plugin sits between that process and native
velocity control. Independent simulation/wall command expiry triggers descent,
including when the ROS process dies. Motors remain enabled in flight and are
disabled only after settled ground, not immediately on command loss. The plugin
publishes measured rotor speeds and safety activation independently of ROS
flight-process liveness. Do not weaken these layers to pass a mission.

## ROS interfaces

`uavN` denotes a configured vehicle; sensor topics use sensor-data QoS.

| Topic/service | Role |
|---|---|
| `/uavN/ground_truth/odometry` | `nav_msgs/Odometry`: measured world ENU pose, body twist, original Gazebo source stamp |
| `/uavN/mpc/pose`, `/uavN/mpc/velocity` | Measured world ENU state, with body twist rotated and source stamps retained |
| `/uavN/scan`, `/uavN/camera/image_raw`, `/uavN/camera/camera_info` | LiDAR and calibrated images |
| `/uavN/target/observation`, `/target/ground_truth` | Selected target input |
| `/uavN/mpc/trajectory` | Source-stamped `trajectory_msgs/JointTrajectory` XY position/velocity/acceleration horizon |
| `/uavN/mpc/prediction` | Predicted path for visualization, never executed as state |
| `/uavN/flight/velocity` | Source-stamped world `geometry_msgs/TwistStamped` from sampling plus position feedback |
| `/uavN/flight/state` | `solo_mpc_interfaces/FlightState`: measured connection/armed/mode/owner/landed status |
| `/uavN/flight/command` | Owner-token `solo_mpc_interfaces/FlightCommand` service for flight lifecycle |
| `/uavN/cmd_vel`, `/uavN/enable` | Exclusive per-UAV flight output: body `geometry_msgs/Twist`, `std_msgs/Bool`, through safety plugin |
| `/uavN/motor_speed`, `/uavN/safety_active` | Actual rotor telemetry `actuator_msgs/Actuators` and independent safety `std_msgs/Bool` |
| `/mpc/status`, `/mpc/setpoint_status`, `/mpc/occupancy_grid` | Controller/publisher diagnostics and visualization map |
| `/mission/start`, `/mission/stop`, `/mission/reset`, `/mission/status` | Persistent mission requests |
| `/mpc/enable`, `/target/run`, `/target/reset` | Publishing and target ownership |
| `/solo/readiness`, `/clock` | Health and simulation clock |

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

Each worker preallocates two 1 MiB, file-backed `pending-0.json` and
`pending-1.json` input journals before readiness. Updating their resident mapped
pages records a complete pending call before IPOPT without directory renames.
They are ordinary JSON padded with spaces; after a worker stops, select the
newest valid `sequence` and replay that file with the same command below.
An independent writer thread atomically publishes completed `solve-*.json`
records through a bounded queue, retaining the configured ring size. Queue
overflow/write/drain errors remain visible in logs, and the two pending input
journals survive worker termination even when a completed record cannot be
written. This protects process-death diagnostics, not host power-loss durability.

Inside the simulation container:

```bash
PYTHONPATH=/workspace/ros_ws/src/solo_mpc OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1   python3 -m solo_mpc.replay_optimizer   /workspace/runs/RUN_ID/optimizer-inputs/uav1/solve-0001.json
```

Replay uses recorded budgets. `--iteration-only` removes the CPU cap for a
separate numerical diagnostic; it does not establish flight or timing success.
`SOLO_MPC_UPSTREAM_PATH` can explicitly select alternative upstream sources for
diagnostics; hash mismatches remain visible in provenance.

### Native migration verification

The tables below preserve historical evidence, including the retired
`large200_5uav_separated` scenario. Its YAML and CLI preset have been removed;
existing run directories remain available for inspection. Current presets are
`small50_3uav`, `large200_5uav`, and `scen1`.

On November 5, 2026, the rebuilt native image completed these full-fleet
missions using default camera target detection, every UAV's LiDAR, and
`--record-video`. No ground-truth target override or one-UAV phase was used.
There is no legacy backend.

| Preset | UAVs | Tracking limit | Actual `runs/` directory | Fleet cycles ready / total | Failed nominal UAV solves |
|---|---:|---:|---|---:|---:|
| `small50_3uav` | 3 | 30 sim s | `20261105T074536Z-19981` | 300 / 300 | 0 |
| `large200_5uav` | 5 | 30 sim s | `20261105T075117Z-11864` | 300 / 300 | 0 |
| `large200_5uav_separated` | 5 | 30 sim s | `20261105T075748Z-16619` | 300 / 300 | 0 |
| `scen1` | 5 | 90 sim s | `20261105T073226Z-7527` | 879 / 883 | 4 |

The commands for each preset were:

```bash
./scripts/dev mpc --preset PRESET --record-video
./scripts/dev mission run --seconds 30  # 90 for scen1
./scripts/dev evaluate
./scripts/dev stop
```

Each mission completed its requested tracking window, then confirmed every
vehicle on settled ground, IDLE, unowned, disabled and disarmed. Final actual
rotor speeds were below `1.1e-24` rad/s. During TRACK, measured altitude stayed
within 2.989–3.005 m for the 3 m presets and 29.962–30.050 m for `scen1`.
Every UAV integrated new LiDAR scans throughout its recorded fleet cycles;
all runs reported upstream `matches_pinned_sources: true`.

| Preset | Minimum sampled obstacle body clearance | Minimum sampled inter-UAV distance | Maximum camera observation age | Finalized video duration / frames |
|---|---:|---:|---:|---:|
| `small50_3uav` | 0.727 m | 1.812 m | 0.118 s | 47.70 s / 954 |
| `large200_5uav` | 0.740 m | 1.360 m | 0.121 s | 48.15 s / 963 |
| `large200_5uav_separated` | 0.456 m | 1.227 m | 0.125 s | 48.65 s / 973 |
| `scen1` | 1.203 m | 17.912 m | 0.126 s | 129.80 s / 2596 |

Every final MP4 decoded without errors and was visually inspected through
takeoff, tracking and landing samples (`reports/video-contact.png`, plus a
fuller-size tracking frame for `scen1`). Videos retained H.264, 1920 × 1080,
20 FPS, scenery/meshes, the following overhead view, LiDAR occupancy panel,
instruments, coverage guides and all UAV camera panels. Initial camera panels
can show the existing waiting placeholders while transport starts.

The four unsuccessful nominal solves in `scen1` were
`Maximum_CpuTime_Exceeded`: two each for UAV 2 and UAV 5. They remain partial
solver failures in telemetry. Delivery exceeded the 0.1 s control period in
3.51% of `scen1` cycles and one separated-start cycle; these are observations,
not new acceptance thresholds. UAV 2 was still joining in SEARCH at the end
of `scen1`; all vehicles in the other presets ended in TRACK. These bounded
runs do not establish complete-route success, later 13 m/s sprint tracking,
repeatability, continuous collision clearance or an exact predictive dynamics
match.

Earlier native attempts remain preserved, not treated as final-adapter evidence:

- `20261105T065144Z-5070`, `20261105T065729Z-5473` and
  `20261105T070535Z-516` preceded acceleration feedforward. Their 30 s missions
  completed, but showed camera SEARCH pauses; the separated run also retained
  21 CPU-cap failures and one infeasible nominal solve.
- `20261105T071438Z-21033` ended on camera SEARCH timeout because desired
  velocity was repeatedly reset near measured velocity without native
  acceleration feedforward. The adapter now compensates the native velocity
  loop; the full fleet was rerun after that correction.
- `20261105T072509Z-18665` tracked after the fix but ended on persistent worker
  deadlines during a concurrent installed-image check. All five workers timed
  out; the exact scheduling/I/O cause was not isolated. The final `scen1` run
  used unchanged settings without concurrent image operations. Both failed
  missions landed all five vehicles, stopped their motors and finalized video.

A separate full three-UAV camera/recording mission,
`20261105T080708Z-25899`, manually paused all three ROS flight processes
(`kill -STOP 87 88 89` inside the container) during tracking. Gazebo independently
latched `safety_active` for every vehicle, descended from 3 m, and stopped all
rotors on settled ground while the three flight processes remained stopped.
The saved `reports/watchdog-ground.json` records rotor maxima below `5.8e-25`
rad/s with safety still latched; independent truth places all vehicles at
approximately 0.195 m. This did not rely on ROS flight commands or predicted
states. After `kill -CONT 87 88 89`, all three flight nodes confirmed
landed/disarmed/disabled/unowned IDLE and normal `./scripts/dev stop` finalized
the 43.0 s / 860-frame video, which decoded and was visually sampled.

This deliberate fault remains a **failed mission**: command replies timed out
and the supervisor's 60 wall-second flight-state watchdog expired while the
nodes were paused (`reports/watchdog-failure.json`). Physical watchdog landing
is not equivalent to a successful supervised mission. Sudden Gazebo/server
death cannot continue physics, and independent watchdog descent at 30 m was
not fault-injected; normal 30 m fleet landing was verified separately.

The native build command, numerical-cache caveat and installed-image checks are
recorded in [Deployment](DEPLOYMENT.md). Host syntax/import/reference checks,
native plugin compilation and `git diff --check` supplement the missions; no
automated tests or test harnesses were added.

### Later native-run failures and local corrections

The later November 5 runs failed during tracking, with successful startup and
subsequent fleet landing. Do not interpret the client's failure exit as a
takeoff or landing failure:

- `20261105T101659Z-8323` and `20261105T132732Z-11708` (small field): all
  workers became permanently unavailable after a roughly 1.46 s fleet cycle.
  Gazebo safety then latched and the mission reported lost flight state. In
  the latter run, all three final optimizer captures show successful solves
  written within 69 ms of capture creation, despite all workers being disabled.
- `20261105T102934Z-13165` (large field): one visible CPU-cap failure preceded
  an all-worker timeout; supervision stopped on persistent planner failure.
  All five last captures succeeded and were written within 0.71 s, below the
  1 s worker budget. Completion-to-pipe delivery cannot be reconstructed from
  those files alone.
- `20261105T083215Z-28774` (`scen1`): slow/expired fleet results interrupted
  leader pursuit, and camera SEARCH exceeded eight simulation seconds. The
  25 m camera half-footprint had little reserve for UAV 3 starting 16.4 m west
  of a target accelerating to 5 m/s. This does not establish an upstream
  numerical failure or prove the complete 13 m/s sprint route is trackable.
- `20261105T082809Z-27690` and `20261105T085239Z-5408` ended stopped without
  a saved mission-failure record. The latter still contains CPU-cap failures;
  neither should be relabeled a successful complete-route mission.

The failed runs reached native readiness; saved final flight states confirm
all vehicles settled, unowned, IDLE and disarmed. The watchdog activations
remain safety interventions during tracking, not failed physical landings.
The three supplied controllers had also been changed to 1.0 pacing while their
comments and guides still specified 0.2. They now request 0.2 again, preserving
all speeds, CPU caps, worker deadlines, source expiry and landing checks.

The local worker collector now drains queued replies even when its coordinator
wakes after the deadline, accepting only worker completions timestamped within
the original phase budget. Truly late workers are still disabled. Obsolete or
flight-invalid cycles retain per-UAV numerical/error diagnostics while marking
every output ineligible for publication. Mission failures freeze their original
controller/setpoint/flight context in `status/mission-trigger.json` before
cleanup updates arrive.

Long 1080p recordings write about 124 MB per simulation second to their recovery
journal. The recorder now synchronizes and evicts journal pages every 32 MiB,
bounding its dirty-page accumulation without removing recovery frames.

Two follow-up runs, `20261105T155955Z-4421` and
`20261105T161044Z-23237`, reproduced all-worker failures even at 0.2 pacing.
Both landed and stopped all three rotors normally. In the latter, a timeout
stack capture caught UAV 2 inside `SolverCapture.finish()`'s `os.replace()`;
the other workers had returned to waiting for jobs when sampled. The final
captures' file modification times reflect the earlier content writes, not
completion of the blocked rename. Thus early successful JSON content alone
cannot establish a timely worker reply. This directly identifies synchronous
diagnostic filesystem work as a worker-deadline hazard; the underlying host
filesystem stall is not fully explained. Timeout stacks now remain in
`logs/worker-<uav>.traceback.log` (captured only after a failed deadline).
Pending mapped journals and the independent completed-record writer remove
those per-solve renames from the flight worker. Solver code and all existing
CPU/wall deadlines remain unchanged.

The final local changes were verified with the following full-fleet camera,
LiDAR and recording missions (run IDs use UTC, November 5; the last checks ran
on November 6 local time):

| Preset | Requested tracking limit | Actual run directory | TRACK cycles ready / total | Failed nominal UAV solves |
|---|---:|---|---:|---:|
| `small50_3uav` | 180 sim s; route goal reached earlier | `20261105T162232Z-11793` | 1519 / 1519 | 0 |
| `large200_5uav` | 130 sim s | `20261105T164344Z-18865` | 1300 / 1300 | 0 |
| `large200_5uav_separated` | 45 sim s | `20261105T170006Z-10965` | 450 / 450 | 0 |
| `scen1` | 90 sim s | `20261105T174634Z-22327` | 891 / 898 | 7 |

Commands were `./scripts/dev build sim`, then for each preset
`./scripts/dev mpc --preset PRESET --record-video`,
`./scripts/dev mission run --seconds LIMIT`, `./scripts/dev evaluate` and
`./scripts/dev stop`. Both initial diagnostic attempts above were retained as
failed runs; a failed CLI exit was followed through automatic landing and
container cleanup before another stack started. The final image includes the
mapped-journal/writer changes. Installed imports were checked separately without
a checkout mount, retaining bundled upstream provenance.

All four final missions completed, landed every vehicle, released ownership and
confirmed disabled/disarmed flight with measured rotor speeds below
`1.3e-24` rad/s. There were no worker timeouts or Gazebo watchdog activations.
Every UAV's final completed capture sequence matched its latest pending input
journal, with 128 completed ring records retained. Upstream source hashes matched
the unchanged pin in every run. The small-field target reached within 0.025 m of
its `[47,23]` goal before the duration limit; the other three checks were bounded
missions, not complete-route evidence.

| Preset | Minimum sampled obstacle body clearance | Minimum sampled inter-UAV distance | Finalized 1080p/20 FPS video duration / frames |
|---|---:|---:|---:|
| `small50_3uav` | 0.724 m | 1.453 m | 179.50 s / 3590 |
| `large200_5uav` | 0.552 m | 1.208 m | 155.60 s / 3112 |
| `large200_5uav_separated` | 0.421 m | 1.203 m | 474.35 s / 9487 |
| `scen1` | 1.012 m | 17.519 m | 138.30 s / 2766 |

Every MP4 decoded successfully and was visually sampled through the preserved
map/state/overhead/all-camera composition (`reports/video-contact.png` and
`reports/video-probe.json`). Recording includes takeoff, landing and idle time
until stack shutdown; the separated recording includes a long post-mission idle
interval. Camera observation age stayed below 0.127 s during TRACK in every run.
The three low-altitude presets remained within 2.970–3.008 m and `scen1` within
29.969–30.048 m during TRACK. Source timestamp sidecars remain intact.

Scen1's seven failed nominal solves remain visible: four CPU-cap failures and
three infeasible solves, affecting UAVs 2 and 4. Later calls recovered, while UAV
2 was still in SEARCH at the end. Four of 898 TRACK cycles delivered later than
the existing 0.1 s control period; the other presets had none. These observations
do not change CPU/deadline acceptance criteria. The fix removes diagnostic
filesystem stalls from worker computation; it does not eliminate every upstream
numerical failure or establish continuous clearance, all handoffs, or complete
13 m/s sprint-route tracking. No automated tests or test harnesses were added.

Retain the actual run directory, resolved settings, source/image identity,
per-UAV solver outcomes, measured odometry, motor speeds and safety status.
Inspect takeoff, tracking, source expiry, landing, settled ground and actual
motor shutdown, plus finalized 1920 × 1080 / 20 FPS video with all map/state/FOV
panels and timestamp sidecars. Process-death safety must remain in Gazebo, not
depend on the mission supervisor or a surviving flight process.

Keep existing worker deadlines, solver CPU caps and command expiry; do not add
new CPU/performance acceptance thresholds or hide unsuccessful solves. Report
wall compute and simulation delivery delay separately, with failures and
limitations. Replay is numerical evidence only, never flight verification.

Earlier backend flights remain in saved `runs/` directories and Git history.
Their timing, measured response, clearance and verified envelopes do not validate
the native dynamics. Installed-image verification must also establish operation
without Git or a host checkout; the check above imports bundled sources/assets,
while the recorded fleet missions use the development mount.

### Scen1 upstream parameter alignment and timing (November 6, 2026)

Run `20261106T042816Z-23613` used headless native five-UAV Gazebo, camera target
input, LiDAR and recording after matching scen1 prediction/formation values
to upstream (see Configuration). Commands were `./scripts/dev mpc --preset
scen1 --record-video`, `./scripts/dev mission run --seconds 90`,
`./scripts/dev evaluate`, and `./scripts/dev stop`. Existing Python/config
changes loaded through the development mount; no native rebuild was required.

All five vehicles reached TRACK and completed supervised landing/disarm;
saved `reports/settled-flight-state.json` records IDLE, landed, unarmed and
actual rotor speeds below 1e-24 rad/s for all five, with safety inactive.
Evaluation retained 895 airborne TRACK cycles, 888 ready and seven unsuccessful
nominal solves: six `Maximum_CpuTime_Exceeded` (UAVs 1, 2, 4) and one
`Infeasible_Problem_Detected` (UAV 2). Subsequent calls recovered. No caps,
deadlines or command expiry were relaxed. Optimizer captures report
`matches_pinned_sources: true`.

Per-UAV solve wall medians were 77.6, 89.7, 78.1, 85.8 and 75.4 ms; p95 values
were 316.7, 306.0, 121.9, 283.6 and 231.8 ms. Full stage statistics are saved
in `reports/timing.json`. Fleet wall p50/p95 was 200.2/382.6 ms, and simulation
delivery delay p50/p95 was 38/75 ms; ten cycles exceeded the 100 ms simulation
control period. Minimum sampled obstacle-body clearance was 1.389 m and
inter-UAV distance 11.876 m. These bounded observations do not establish
sub-70 ms solves, full-route/sprint completion or a measured dynamics envelope.

Normal stop finalized the video. Takeoff/tracking frames retained all five
FOVs, map, state rows and separated timings; finalized video and timestamp
sidecars remain in the run directory. The upstream submodule was unchanged.

### Scaled scen1 configuration (November 6, 2026)

The local scen1 preset now uses a 200 × 200 m map at 6 m altitude. All five
starts, 33 rectangles, 32 circles and 27 target waypoints retain exactly one
fifth of the former coordinates and dimensions (maximum numeric difference
from division by five below 3e-14 m). The colleague-owned submodule is unchanged.
See Configuration for scaled sensing, formation, route and physical-Iris
exceptions. No native code, CPU caps, worker deadlines or expiry guards changed.

Initial camera/LiDAR/recording run `20261106T090320Z-4437` used literal 2 m/s
and 1 m/s² prediction bounds. Repeated follower CPU-cap failures (39 failed
nominal calls) led to an explicit `mission stop` after roughly 15 tracking
seconds; the originally requested 90-second window was not completed. All five
landed, released ownership and disarmed, and normal stop finalized the video.
This shortened attempt is tuning evidence, not a successful 90-second check.

Retry `20261106T090758Z-17758` used 2.2 m/s and 3 m/s² prediction bounds,
with target process-noise and velocity-trust thresholds scaled to 0.4 m/s²
and 0.3 m/s. Commands were:

```bash
./scripts/dev mpc --preset scen1 --record-video
./scripts/dev mission run --seconds 90
./scripts/dev evaluate --run-dir /workspace/runs/20261106T090758Z-17758
./scripts/dev stop
```

The retry completed the requested tracking window with camera input and all
five native vehicles. All five reached formation TRACK by simulation stamp
71.68 s, approximately 59 s after tracking began, and remained in TRACK through
the end. Camera observation age stayed at or below 0.126 s; measured TRACK
altitude ranged from 5.901 to 6.009 m. The 875 completed airborne TRACK cycles
included 830 ready cycles and 47 failed nominal UAV solves: 44 CPU-cap failures
(UAV 1: 2, UAV 2: 21, UAV 4: 21), plus three infeasible solves (UAV 2: 2,
UAV 4: 1). Later calls recovered. These failures remain visible in telemetry;
a completed supervised mission does not mean every optimization succeeded.

The run is bounded early-route evidence, including rendezvous and the first
bend. It does not establish complete-route or later 2.6 m/s sprint success,
repeatability or continuous collision clearance. The scaled obstacles leave
some tight approaches; inspect sampled body clearance alongside the video.

Final evaluation reported minimum sampled obstacle-body clearance of 0.136 m
and inter-UAV distance of 1.591 m. Fleet wall p50/p95 was 150.8/487.4 ms;
simulation delivery delay p50/p95 was 28.0/96.3 ms, with 4.8% of cycles over
the existing 0.1 s simulation control period. No performance caps were relaxed.
Saved `reports/settled-flight-state.json` confirms every vehicle IDLE, landed,
unowned, disarmed and disabled, with actual rotor speeds below 8e-25 rad/s.
No Gazebo safety watchdog activated during the run.

Normal stop finalized a 119.4 s / 2388-frame 1920 × 1080 / 20 FPS video.
Decoded frame counting completed without errors; takeoff, tracking and landing
samples retain the overhead, occupancy, five state rows, all five camera panels
and coverage guides (`reports/video-contact.png`, `tracking.png`, `takeoff.png`,
`landing.png`, `video-probe.json`). Source timestamp sidecars remain intact.
Provenance reports `matches_pinned_sources: true`. The container is stopped.
`git diff --check` passed; no automated tests or harnesses were added.
