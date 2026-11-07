# Development

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Configuration](CONFIGURATION.md) · [Deployment](DEPLOYMENT.md)

Read [AGENTS.md](../AGENTS.md) before editing. Keep numerical code independent
of ROS and preserve measured-state control and independent flight supervision.

## Architecture

`scripts/dev` starts Docker Compose; `solo_mpc/launch/mpc.launch.py` owns Gazebo,
one native ROS flight process per UAV, readiness, bridges, frame/target
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
| `core/workers.py`, `cycles.py` | Reusable persistent process transport, phase deadlines, asynchronous cycles |
| `core/parallel_controller.py` | MPC worker payloads, map barrier, frozen predictions, guidance and solve diagnostics |
| `controller_node.py`, `frame_node.py`, `target_node.py` | ROS input/output, odometry twist rotation, ground-truth target publication |
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

The ROS-free `Algorithm` protocol takes a `CycleInput` and returns a
`CycleResult`. The only factory selection remains `algorithm: mpc`.
`MPCAlgorithm` composes the MPC coordinator rather than inheriting its planner
and solver internals. It translates existing numerical outputs at this boundary;
MPC mathematics, guidance, limits and retained stopping behavior are unchanged.

`CycleInput` carries measured `VehicleState` values, projected `ScanRays`, a
common source epoch, seers and a target-estimate snapshot. Pose and velocity
source stamps remain distinct. `CycleRunner` deep-copies the entire input before
submission; frozen contract containers express snapshot ownership, but do not
make NumPy arrays or dictionaries deeply immutable. Numerical code must treat
these owned inputs as read-only. No ROS messages enter the core.

Each `VehicleResult` separates an `ExecutableTrajectory` (world XY positions,
velocities and commanded accelerations) from a `NeighborPrediction` (planning
state rows). Both carry source epoch and timestep. A prediction alone does not
authorize execution. `nominal_success`, `fallback_used` and
`publication_eligible` have separate meanings: a retained stop can be eligible
with a failed nominal solve, while an otherwise successful solve can become
ineligible after expiry or a flight-state change. Collection asks the MPC adapter
to revalidate retained stops against current measured state using the original
position/velocity/acceleration tolerances. Rejection changes publication
eligibility without erasing the nominal outcome or fallback selection.
Here `fallback_used` denotes selection of the retained stopping trajectory;
it does not describe an internal numerical solver retry or confirm execution.

`CycleResult` includes a copied coordination metadata snapshot alongside vehicle
results. Solver/stage details remain algorithm diagnostics, outside trajectory
fields. ROS telemetry retains existing names including `nominal_ok`, and adds
`fallback_used` and `publication_eligible`. ROS 2 Jazzy still owns scheduling,
guard notifications, input validation, trajectory/path/map/status publication,
setpoint sampling and flight/mission supervision. Headers retain the exact
captured ROS input epoch, including its nanoseconds, rather than completion time.

`WorkerPool` owns spawn, pipes, startup/phase barriers, cycle matching, deadline
rejection, disconnected-worker disabling and bounded shutdown. MPC supplies the
worker entry point, phase payloads, ordered-solve policy and timeout traceback
observer. The generic runtime imports no MPC planner, solver or capture module.
The MPC coordinator still freezes fleet predictions before mapping/guidance,
requires all maps, and commits predictions only after the solve barrier. Each
worker retains its own numerical state. Development source loading and installed
package discovery use the same modules; bundled upstream sources are unchanged.

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
| `/target/progress` | JSON scenario waypoints reached by the target route clock |
| `/mission/start`, `/mission/stop`, `/mission/reset`, `/mission/status` | Persistent mission requests |
| `/mpc/enable`, `/target/run`, `/target/reset` | Publishing and target ownership |
| `/solo/readiness`, `/clock` | Health and simulation clock |

## Verification and optimizer replay

Do not write automated tests. Run a representative mission, inspect measured
telemetry, logs and evaluation; see [Simulation](SIMULATION.md). One
`./scripts/dev run --preset PRESET --seconds N --record-video` command now
replaces the earlier `mpc`, `mission run`, `evaluate` and `stop` sequence
recorded in the dated verification sections below.
Ground-truth target input keeps solver changes isolated from perception; add
recording when visual behavior is relevant. Preserve the specific
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
Camera detection has since been removed; current runs always use ground truth.
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

At that revision, long 1080p recordings wrote about 124 MB per simulation
second to a raw recovery journal, with page eviction every 32 MiB. The current
recorder replaces that journal with compressed segments and a bounded one-second
RAM retry window; see the recording section in the simulation guide.

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

## Streaming recording verification (November 6, 2026)

Run `20261106T110701Z-13502` used the default full three-UAV fleet, camera
input and native Gazebo physics:

```bash
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

During flight, the recorder's NVENC subprocess was deliberately killed with
`docker compose exec -T sim pkill -KILL -f '^ffmpeg .*h264_nvenc'`.
It switched to two-thread CPU encoding and retried the current RAM segment.
The CPU encoder was subsequently killed with the equivalent `libx264` pattern.
Recording continued, and normal stop published a **partial** 47.4-second,
948-frame, 1920 × 1080 / 20 FPS MP4: 941 retained frames and seven held slots
at frames [140, 147). The GPU fallback occurred at input frame 58. No raw-frame
files were created. The final video was 44,426,004 bytes. Compressed attempts
and recovered segments remain under the run's `.encoding` directory.

Full FFmpeg decoding completed without errors. Frame PTS matched `index / 20`
exactly, and every sidecar simulation stamp advanced by 50,000,000 ns, including
the gap. Every published source-stamp dictionary matched its selected original
input. `reports/video-contact.png` was visually inspected at takeoff, gap
boundaries, tracking and landing; overhead, occupancy, state instruments, three
FOV panels and coverage guides remain present. `reports/video-probe.json`
records the decoded count, dimensions, rate and PTS check.

All three UAVs ended IDLE, landed, unowned and disarmed, with actual motor
speeds below 6e-25 rad/s and no safety watchdog activation. Provenance matched
the pinned upstream sources. All 897 TRACK nominal solves succeeded. Evaluation
reported fleet wall p50/p95 133.8/244.1 ms and 62.5% over 100 ms wall time;
simulation delivery delay p95 was 39 ms, with none over the existing 0.1-second
simulation period. No limits were relaxed.

The initial run exposed FFprobe advertising 40 FPS for a one-frame fragmented
segment whose average rate and duration were actually 20 FPS / 0.05 seconds.
Verification now uses average rate for that special case, preserving that
valid frame without a spurious encoder-failure warning.

Follow-up `20261106T111257Z-271` restarted the stack with this correction and
used the same commands with `mission run --seconds 5`. Its healthy NVENC path
finalized all 447 frames (22.35 seconds) with no fallback, missing frames or
held slots. Decoded PTS again matched the 20 FPS grid exactly, with no decoder
errors. Takeoff, tracking and landing samples in `reports/video-contact.png`
were visually inspected. All three UAVs landed/disarmed, motors fell below
4e-25 rad/s, and the stack stopped normally. Evaluation retained wall timing
p50/p95 194.7/307.5 ms (100% over 100 ms wall time); simulation delivery p95 was
59 ms, with none over the existing simulation control period.

A frozen copy of that run's in-flight compressed files was recovered separately
under `reports/interrupted-snapshot/` using
`PYTHONPATH=ros_ws/src/solo_mpc python3 -m solo_mpc.core.video SNAPSHOT --recover`.
The command retained 125 of 128 journaled frames, held the three missing tail
slots and reported `partial`, `interrupted=true`; the live recorder continued
independently. This is snapshot recovery evidence, not a real recorder or host
power-loss experiment. GPU initialization denial, full disks, prolonged encoder
stalls, complete recorder death and the larger fleet presets were not exercised
in this recording change. No automated tests or harnesses were added. Python
compilation, import/caller checks and `git diff --check` passed.

## Runtime contracts and worker extraction verification (November 6, 2026)

Before editing, the clean tree at
`9b192a05200839e656af1f14ae3f0638d8645bf8` was preserved under
`artifacts/runtime-contract/before/`: repository bundle, commit/status/diff,
submodule pin, image list, resolved inputs and provenance snapshots. The existing
`runs/20261106T114230Z-29772/` was retained with a file-hash manifest. Its 7,481
completed airborne cycles already included 43 CPU-cap failures and two infeasible
nominal solves. Earlier failures and recording limitations above remain evidence.
The original image remains tagged `solo-simulation:runtime-contract-baseline`.

Three fresh baselines completed before the live source tree changed. All six
missions below used full fleets, default camera target input, per-UAV LiDAR and
recording, with identical resolved fleet/scenario/controller/bridge/MPC inputs
within each pair. Every airborne scan reported 360 usable rays per UAV.

| Preset | Tracking limit | Baseline directory under `runs/` | Refactored directory under `runs/` | Ready / completed airborne cycles, before → after |
|---|---:|---|---|---|
| `small50_3uav` | 30 sim s | `20261106T143346Z-30131` | `20261106T145733Z-4706` | 300/300 → 300/300 |
| `large200_5uav` | 30 sim s | `20261106T143828Z-26772` | `20261106T150207Z-20156` | 300/300 → 299/299 |
| `scen1` | 90 sim s | `20261106T144337Z-21980` | `20261106T151208Z-897` | 832/876 → 821/870 |

Commands for each preset were:

```bash
git submodule update --init --recursive
./scripts/dev mpc --preset PRESET --record-video
./scripts/dev mission run --seconds SECONDS
./scripts/dev evaluate
./scripts/dev stop
```

After the baselines, `./scripts/dev build sim` completed normally. The final
scen1 mission used `mission start --seconds 90`, followed by supervisor status
inspection through terminal `completed`, evaluation and normal stop. Its original
completed status is also saved in `reports/runtime-contract-completed-mission.json`.
The host command sequence exited 143 after the refactored large mission had
completed; evaluation and normal stop were explicitly completed in a subsequent
command. Its longer video includes the additional grounded wait. No simulation
was force-stopped. Command logs live under `artifacts/runtime-contract/`.

Neither small nor large had failed nominal UAV solves. Scen1 had 45 before
(40 CPU-cap, five infeasible) and 50 after (49 CPU-cap, one infeasible).
Afterward UAV 2 had 21 CPU-cap failures and one infeasible solve, UAV 4 had 27
CPU-cap failures, UAV 5 had one, and UAVs 1/3 had none. Later solves recovered.
One retained stopping trajectory was selected and rejected by the moved
measured-state revalidation: `fallback_used: true`, `publication_eligible: false`,
`stop_plan_measured_state_changed`. The baseline also had one such rejection.
No accepted stopping-trajectory publication was observed in these windows.
There were no worker timeouts, obsolete-cycle results or flight faults in the
completed airborne cycles; the partial-solver-failure records remain visible.

| Preset | Fleet wall p50/p95 ms, before → after | Simulation delivery p95 ms, before → after | Delivery over existing control period, before → after |
|---|---|---|---|
| `small50_3uav` | 63.1/80.2 → 73.0/89.8 | 15.1 → 17.0 | 0% → 0% |
| `large200_5uav` | 94.3/122.1 → 272.4/387.7 | 23.1 → 53.0 | 0% → 0% |
| `scen1` | 152.2/417.4 → 378.7/813.0 | 81.8 → 128.0 | 4.68% → 7.70% |

The post-change large/scen1 runs overlapped an unrelated host numerical process
using approximately 22 CPU cores. Observed simulation pacing dropped, and solver
wall time exceeded process CPU time. The process was left untouched; observations
are preserved in `host-load-observation.txt` and `host-load-scen1.txt`. Baselines
used image `sha256:261530a69f7532f73db13313b694429a8e35bba0430f0a28056d635b951a3c19`;
post-change flights used rebuilt image
`sha256:f72f491c85d5d0279c26507df5f2749211a206e3ead54ec3d6b21966638ba97a`.
These are observed timings under different host load and rebuilt runtime inputs,
not an isolated benchmark or proof of behavioral/numerical equivalence. Existing
CPU caps, deadlines, command expiry, pacing settings and performance budgets were
not relaxed.

Minimum sampled obstacle-body clearance before → after was 0.731 → 0.715 m
(small), 0.725 → 0.656 m (large), and 0.278 → 0.192 m (scen1). Minimum inter-UAV
distance was 1.812 → 1.818 m, 1.350 → 1.340 m, and 1.606 → 1.550 m respectively.
Every vehicle in every run ended IDLE, landed, unowned, unarmed and disabled;
final actual rotor speeds were below 2e-24 rad/s. Flight journals contain no
safety-watchdog activation or fault. These bounded windows do not establish
continuous clearance, full-route/sprint completion, repeatability or a new
process-death safety experiment.

All recordings finalized at 1920 × 1080 / 20 FPS. Decoded baseline/post frame
counts were 920/922 (small), 934/1093 (large), and 2369/2409 (scen1). Full FFmpeg
decoding returned no errors. Visually inspected takeoff/tracking/landing samples
retain overhead, occupancy, state instruments, coverage guides and every UAV FOV.
All sidecars retained 50 ms slots and original source-stamp dictionaries, without
held/dropped frames. The refactored small recording reported a CPU encoder
fallback on the final two-frame segment's format check (`r_frame_rate: 40/1`,
`avg_frame_rate: 20/1`); all 922 frames were retained. Recording code/layout was
unchanged. Other five recordings reported no encoder fallback.

Controller provenance and sampled optimizer captures for every UAV matched
upstream pin `db96e52b89c24a559271b1a24e36c6d3f9c53873`, with unchanged numerical
controller, guidance, solver adapter, target estimation and upstream hashes.
A sampled ROS trajectory had source stamp 27.320 s, matching its cycle, while
reporting occurred at 27.333 s; world XY references retained 0.1 s offsets.
No-mount/no-network installed-image imports loaded the new modules from
`/opt/solo_ws/install` and unchanged bundled upstream sources with
`matches_pinned_sources: true` (`installed-imports.json`). This is installed
packaging/provenance evidence; the six flights used development source mounts.

Per-run `reports/runtime-contract-summary.json`, `runtime-contract-video.json`,
`runtime-contract-decode.log` and `runtime-contract-contact.png` retain the
inspection results. `artifacts/runtime-contract/comparison.json` includes resolved
input hashes, per-UAV timing/outcomes, flight/motor journals, sampled optimizer
provenance and timestamp-sidecar audits. Compilation, import/caller checks and
`git diff --check` passed. No automated tests, harnesses, alternative algorithms,
configuration changes or upstream edits were added. The simulation is stopped.

## Single-command run verification (November 7, 2026)

`./scripts/dev run` replaced the separate `mpc`, `mission run`, `evaluate` and
`stop` host commands. Default three-UAV fleet, camera input and recording:

| Case | Actual `runs/` directory | Exit | Result |
|---|---|---:|---|
| `run --seconds 30 --record-video` | `20261107T060118Z-536` | 0 | Landed IDLE/disarmed/disabled, rotors < 4e-25 rad/s; 921/921 frames finalized; evaluated; container stopped |
| Ctrl+C (group SIGINT, twice) during tracking of a 60 s run | `20261107T055120Z-21378` | 130 | Cancel file landed the fleet; 350/350 frames finalized; evaluated; container stopped |
| `docker kill` during tracking of a 60 s run | `20261107T055918Z-22364` | 137 | Offline recovery published a partial MP4 (345/349 frames); evaluated; container stopped |
| Missing `--scenario` file | `20261107T055651Z-27702` | 1 | Launch failed before readiness; previous run untouched; container stopped |

A Ctrl+C sent during an earlier failure cleanup was ignored and cleanup completed.
`matches_pinned_sources` was true. No GUI (`--gui`) run was performed.
