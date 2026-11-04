# Development

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Configuration](CONFIGURATION.md) · [Deployment](DEPLOYMENT.md)

Read [AGENTS.md](../AGENTS.md) before editing. Keep numerical code independent
of ROS and preserve measured-state control and independent flight supervision.

## Architecture

`scripts/dev` starts Docker Compose; `solo_mpc/launch/mpc.launch.py` owns Gazebo, one SITL/MAVROS
pair per UAV, readiness, bridges, frame/perception/target nodes, the controller,
independent setpoint publisher, and mission supervisor. The GUI is a separate
client. Each persistent UAV worker owns its map, planner, corridor, solver cache,
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
| `solo_bringup/` | Fleet validation, run snapshots, MAVROS bootstrap, readiness and flight ownership |
| `solo_mpc/gazebo/` | Native world-stabilized camera and coverage plugins |

The launch resolves fleet/scenario/controller inputs once for world generation,
SITL overrides, bridges and ROS nodes. `solo_mpc/world.py` builds the Iris fleet
and sensor world together; `solo_bringup/launching.py` owns MAVROS configuration
and process lifecycle callbacks. Every run is fresh; old sensor/alignment state
cannot be resumed. `core/visual_style.py` shares the camera framing and fleet
palette between the GUI, world and recorder.

`TeamController.step(states, scans, stamp, seers)` has no ROS imports and never
advances a simulated replacement for the measured vehicle. Workers freeze a
common prediction snapshot before solving. Spawn avoids inheriting DDS/native
solver state. Late results are rejected; timed-out workers stop until controller
restart. Independent publishing and supervision remain able to expire references
and request LAND.

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
Compare complete airborne, armed/GUIDED records using identical resolved inputs
and recording settings; solver work and host load can vary between flights.

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
numerical conditioning and optional measured-overspeed recovery (enabled by the
supplied controller), preserving the objective, other active constraints,
stopping branch, and upstream solve routine. It checks constraint structure and
fails explicitly if an upstream change invalidates the adjustments. Optional SQP/IPOPT fallback also
uses the adapter. Do not restore copied solver/dynamics implementations.

Each local UAV now owns a bounded `SolverCache` of unchanged upstream problems.
Selection uses exactly `max(1, hard face count, cost face count)`; neither
polytope is truncated or simplified, and both nominal and full backup horizons
remain enabled. Sizes 4–8 and the configured `CORRIDOR_MAX_FACES` (normally 16)
are constructed and solved with synthetic stationary inputs before worker
readiness, including any SQP/IPOPT fallback. Initialization failure prevents
readiness. That configured size is a preinitialized capacity, not a minimum for
every solve. Anticipated instances stay resident; two additional LRU slots hold
other exact counts (eight instances per UAV with the supplied configuration).
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

The default requested physics rate is 0.2× wall time. Gazebo retains the original
integration step; each vehicle's JSON plugin explicitly enables `lock_step` and
`no_time_sync`, making FCU time follow physics without a second SITL wall pacer.
Control and sensor rates, prediction horizons, and video sampling stay in
simulation seconds. Worker/CPU deadlines and wall freshness remain independent.
Evaluation reports both wall computation time and simulation input-to-result
delay; the former can exceed 100 ms while the latter meets a 0.1 s control period.

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

The 0.2× physics configuration was checked in ground-truth/video run
`20261104T103307Z-9632` and five-UAV camera/video run `20261104T104137Z-10688`.
Commands were `./scripts/dev mpc --ground-truth-target --record-video` and
`./scripts/dev mpc --preset large200_5uav --record-video`, each followed by
`mission run --seconds 150`, `evaluate`, and normal `stop`. All 313 / 271 airborne
cycles were ready and all 939 / 1,355 nominal solves succeeded. Observed rates
were 0.188× / 0.159×; simulation input-to-result delay p95 was 13 / 25 ms and
maxima were 16 / 31 ms, below the unchanged 100 ms simulation period. Wall
compute p95 remained 85.9 / 135.2 ms. Minimum sampled obstacle body clearance
was 0.787 / 0.827 m and UAV separation was 1.889 / 1.330 m. All vehicles landed
and disarmed before shutdown. All 384 / 640 retained captures matched unchanged
upstream pin `3f71d4c` and adapter. Neither run logged worker timeouts or Python
tracebacks; known SITL SIGINT exits and MAVROS SIGTERM escalation remain visible
after disarm. Both finalized recordings decoded fully at 1920 × 1080 / 20 FPS
(905 / 804 frames), with exactly 50 ms of simulation time per frame and unique
overhead frames. Reviewed takeoff, tracking and landing images retain all
state/FOV panels. Evidence is in each run's `reports/slow-physics-verification.json`,
evaluation, video report, sampled PNGs and telemetry; command/container logs are
in `artifacts/slow-physics/`. These are partial-route flights without artificial
CPU saturation; finite slowdown cannot guarantee deadlines under arbitrary load.

Fleet completion notification was compared with instrumented polling baseline
`20261103T135411Z-4686` and guard-condition run `20261103T135845Z-31238`.
Both used `mpc --preset small50_3uav --ground-truth-target`,
`mission run --seconds 60`, `evaluate --run-dir /workspace/runs/RUN_ID`, mission
status and normal `stop`, without video. Image, resolved fleet/scenario/settings,
MPC configuration and unchanged upstream pin `3f71d4c` matched. The baseline had
the same timing instrumentation; only controller completion handling and cycle
shutdown differed between loaded Python sources.

| Complete airborne armed/GUIDED cycles | Polling | Guard condition |
|---|---:|---:|
| Fleet cycles / successful nominal calls | 611 / 1,833 | 617 / 1,851 |
| Completion-to-collection median / p95 | 9.69 / 18.76 ms | 0.32 / 0.74 ms |
| Total cycle median / p95 | 99.83 / 120.34 ms | 87.64 / 107.26 ms |
| Fleet cycles over 100 ms | 41.1% | 13.1% |
| Per-call solver CPU median / p95 | 58.77 / 79.12 ms | 58.82 / 76.44 ms |
| Sampled minimum body clearance / separation | 0.756 / 1.771 m | 0.766 / 1.777 m |

Median notification/dispatch delay fell 96.7%; cycle median/p95 fell
12.2%/10.9%. These were separate flights with approximately real-time clocks,
not identical physical/solver trajectories. The 100 ms fleet deadline remains
unmet. Each run rejected one completed cycle after flight mode changed during
landing; every UAV landed/disarmed and released ownership. Across all 659/668
completed cycles there were no duplicate source epochs or overlapping cycles.
Each run retained 384 captures matching upstream and the local adapter; all
128 complete captured fleet epochs had identical common targets and named
neighbor predictions. Independent publisher, worker barrier, numerical control
and mission supervision sources were unchanged. The event run had no Python
traceback, notification error or worker timeout; shutdown retained the known
SITL SIGINT exits and MAVROS SIGTERM escalation after disarm.

Evidence is in each run's `reports/completion-latency.json`, evaluation, source
metadata, optimizer captures, telemetry and mission status; the event run also
has `reports/completion-comparison.json`, `mission-final.json`, `compose.log`,
and the recorded implementation patch. No automated tests were added or run.
Future exceptions, a paused clock and shutdown during a solve were reviewed in
code, without injected flight faults. Installed-image behavior and full-route
coverage were outside this check.

Camera run `20261103T140241Z-23997` used
`mpc --preset large200_5uav_separated`, `mission run --seconds 60`, evaluation,
mission status and normal stop, with GPU rendering and no video. At 0.279
simulation seconds per wall second, 197 airborne cycles had completion delay
median/p95/max 0.26/1.00/1.48 ms and total cycle median/p95/max
163.31/305.73/388.74 ms. All five landed/disarmed and released ownership.
UAV 5 reached the preset's unchanged 0.35 s CPU cap ten times and recovered:
975/985 nominal calls succeeded; all ten failed captures were preserved under
`reports/cpu-limit-captures/`. Minimum sampled body clearance/separation was
0.423/1.715 m. This short, slow-clock run did not bring all distant followers
into TRACK and does not establish the five-UAV latency target.

Its `command-bag/` records a 120-wall-second window from takeoff through part of
landing. All 252 published source epochs matched completed-cycle telemetry;
242 had all five reference/trajectory pairs with their common source epoch,
and ten omitted the failed UAV 5 plan. There were no duplicate plan messages.
The independent publisher emitted 1,665 setpoints with application timestamps
distinct from plan epochs; its unchanged guards reported 149 command-limit
and seven expiry samples, retaining their guarded output. Bag receipt time uses
simulation time and is not FCU actuation latency. All 128 complete retained
optimizer epochs shared targets and named neighbor predictions, and all 640
captures matched upstream/adapter sources. No cycles overlapped. See
`reports/completion-latency.json`, `command-source-epochs.json`,
`solver-failures.json`, `mission-final.json`, and the bag. The recorder's
timeout exit 124 was expected; SIGINT finalized its data before normal stop.

Solver caching was compared with the same small preset, configuration, image
and upstream pin `3f71d4c`: baseline `20261103T051741Z-536` and cache run
`20261103T052649Z-94` both used `mpc --ground-truth-target --record-video`,
`mission run --seconds 30`, `evaluate`, mission status and normal `stop`.

| Airborne armed/GUIDED samples | Fixed 16 faces | Cached exact sizes |
|---|---:|---:|
| Successful nominal calls | 726/726 | 732/732 |
| Per-call solver CPU median / p95 | 71.9 / 98.6 ms | 47.5 / 68.1 ms |
| Fleet cycle median / p95 | 110.2 / 147.9 ms | 85.8 / 106.5 ms |
| Fleet cycles over 100 ms | 79.3% | 7.4% |
| Sampled minimum body clearance / separation | 0.828 / 1.766 m | 0.782 / 1.834 m |

Median solver CPU fell 34.0% and fleet p95 fell 28.0%; the 100 ms fleet goal
remains unmet. These are separate flights with similar clock rates (0.706/0.710
simulation seconds per wall second), not identical physical trajectories.
All UAVs landed/disarmed and released ownership. The cache run used 4–7 faces;
all 384 retained captures matched upstream, and all 128 complete fleet epochs
had identical shared target/neighbor inputs. All 16 retained instance switches
reset backup guesses. Publisher, coordination, cycle and supervision sources
were unchanged. Both recordings decoded fully and sampled tracking/landing
frames were inspected.

Paired replay of 36 frozen baseline inputs, with recorded CPU caps and identical
nominal/backup guesses, succeeded for both sizes: CPU median 63.4 → 41.8 ms,
p95 77.8 → 55.8 ms, maximum state/control difference below `7e-9`, and maximum
constraint violation `1.35e-7`. Native preinitialization of six sizes took 0.99 s
in a separate diagnostic. Synthetic replay with repeated cost faces exercised
9/10/11/17-face selection and LRU eviction, stayed within eight resident sizes,
and retained the 25-step backup. This covers cache behavior numerically, not
flight through unusually complex corridors. See the cache run's
`reports/solver-cache-comparison.json`, `paired-replay.json`,
`cache-overflow-replay.json`, `cache-flight-evidence.json`, `replay-exact.txt`,
and the baseline's `reports/replay-original.txt` and `cache-preflight.json`.

Camera/video run `20261103T054228Z-3847` used the separated five-UAV preset,
`mission run --seconds 180`, evaluation and normal stop. All five joined TRACK,
landed/disarmed and released ownership. Of 565 airborne cycles, 564 were ready;
2,824/2,825 nominal calls succeeded. UAV 5 reached the unchanged CPU cap once
at cycle 183, then recovered; its capture had already been overwritten by the
bounded ring, so that failure is supported by telemetry rather than replay.
Solver CPU median/p95 was 62.5/116.8 ms; fleet median/p95/max was
151.6/225.1/393.9 ms. The first unanticipated nine-face instance took 156.4 ms
to construct/select and produced a 359.7 ms fleet cycle; subsequent nine-face
cycles reused it. Minimum sampled body clearance/separation was 0.429/1.273 m.
All 640 retained captures matched upstream, all 128 complete fleet epochs had
coherent shared inputs, all 25 retained switches reset backup guesses, and all
captured backups retained 25 controls. Its recording decoded fully; sampled
joining, tracking and landing frames were inspected, and a retained capture
replayed exactly under its recorded CPU cap. See `reports/cache-flight-evidence.json`,
`solver-failures.json`, `nine-face-cycles.json`, `mission-final.json` and video
frames. This partial route has no matched fresh five-UAV baseline; neither
replay nor sampled geometry proves continuous safety or the 100 ms goal.

Independent LiDAR integration was checked with unchanged upstream pin `3f71d4c`
and unchanged solver constraints/expiry/supervision. Ground-truth/video run
`20261101T095243Z-13427` used `mpc --ground-truth-target --record-video`,
`mission run --seconds 30`, `evaluate`, and normal `stop`: all 240 airborne
cycles were ready and all 720 nominal solves succeeded. All UAVs landed/disarmed
and released ownership. Compute p95 was 144.2 ms, above the 100 ms target.

Camera/video run `20261101T095852Z-19425` used
`mpc --preset large200_5uav_separated --record-video`,
`mission run --seconds 180`, `evaluate`, and normal cleanup. All five UAVs joined,
landed/disarmed and released ownership. Of 514 airborne cycles, 508 were ready,
four had visible solver CPU-cap failures, and two rejected stale manually
republished snapshots. Recorded native scans and measured poses showed up to
14.2 degrees of tilt and 1.47 m endpoint height displacement. In 31 airborne
cycles the former height band would have retained fewer than 70% of a UAV's
rays; the new path retained these rays and continued mapping/MPC. Sampled minimum
body clearance/separation were 0.402/1.458 m; compute p95 was 335.9 ms. The later
camera restart could not reacquire the displaced target and timed out; explicit
cleanup confirmed disarm before normal stop. The auxiliary scan recorder stopped
on SQLite locking during concurrent analysis; its saved database passed integrity
inspection and `ros2 bag reindex`. This limits the recorded sensor interval, not
the complete mission telemetry. See `reports/tilt-evidence.json`,
`solver-evidence.json`, `evaluation-180.json`, and the saved scan bag.

Fresh partial/empty scans were checked in ground-truth/video run
`20261101T101149Z-13454`, using `mission start --seconds 45` after launch.
Stock `ros2 topic pub` commands used simulation timestamps and reliable QoS at
100 Hz for 2,000 messages per UAV: UAV 2 received 90 no-return rays out of 360
with 1 m range and NaNs elsewhere; UAV 3 received only invalid ranges (NaN,
zero, negative infinity, below minimum and above maximum). Native scans continued
at 10 Hz. All 129 partial-scan cycles updated UAV 2's map. All 123 empty-scan
cycles left UAV 3's map unchanged, while the other UAVs mapped and all three
nominal solves succeeded. All 349 airborne cycles were ready, with 1,047/1,047
nominal successes, 0.691 m sampled body clearance, 1.797 m separation and
128.2 ms compute p95. Evaluation and normal stop followed confirmed landing,
disarm and ownership release. Exact publication inputs and outcomes are in
`reports/partial-uav2.yaml`, `empty-uav3.yaml`, `scan-evidence.json`,
`solver-evidence.json`, and `mission-final.json`.

All retained captures (384/640/384 respectively) matched the unchanged upstream
pin and local adapter. The recordings decoded completely and sampled frames
were inspected. These runs cover interleaved dropouts and short no-return partial
scans, not permanent sensor loss, arbitrary sparse obstacle returns, or the full
routes. Native scans in the recorded tilt interval did not reach the ground;
no noisy ground-hit rejection claim follows from this evidence. The 2D height
ambiguity and unbounded Gaussian noise remain as described in
[Configuration](CONFIGURATION.md#sensors-and-target-input). Normal shutdown
retained the known SITL SIGINT exits and MAVROS SIGTERM escalation after disarm.

The slower separated preset sets scenario `VMAX` to 1.2 m/s (previously 2.2),
with target speeds 0.4 m/s on the first two legs and 0.6 m/s thereafter.
The shared large controller leaves `VMAX` to each scenario; the original large
scenario/generator retain 2.2 m/s, and the small preset still resolves to 1.2 m/s.
Acceleration/braking, solver budgets, flight supervision and expiry are unchanged.
Headless camera/video run `20261029T154718Z-23044` completed 450 wall seconds of
tracking (129.6 simulation seconds, 69.0 m of target travel), then landed and
disarmed all five UAVs with ownership released before normal stop. All five first
entered TRACK after 31.9 simulation seconds and stayed TRACK for all 976 subsequent
airborne controller cycles. UAV 1 remained leader with fresh target input in all
1,323 airborne samples; all 1,061 sampled leader images contained the marker.

Measured median UAV speeds fell from 0.977–1.048 to 0.569–0.628 m/s compared with
`20261029T152520Z-17272`. The largest measured speed fell from 2.562 to 1.851 m/s;
followers still overshot the prediction bound, reaching 1.699–1.851 m/s, while
the leader peaked at 0.735 m/s. Minimum sampled separation increased from 1.111
to 1.309 m. Minimum obstacle body clearance was 0.151 m (UAV 5 during approach),
with no sampled footprint overlap. Of 1,323 airborne cycles, 1,249 were ready,
62 rejected tilted scans and 12 had partial nominal failure; 6,292/6,305 nominal
calls succeeded. All 13 failed calls reached the unchanged CPU cap before full
rendezvous; no nominal failures occurred after joining. Fleet compute p95 was
294.8 ms, above the 100 ms target and the previous run's 264.2 ms. These speed
changes reduce measured motion peaks; computation remains a limitation.

All 640 retained captures matched the unchanged upstream pin and adapter.
The 164.95 s recording decoded all 3,299 frames; reviewed closest-approach and
tracking frames show the slower fleet with complete state/FOV panels. No worker
timeouts or Python tracebacks were observed. Known SITL SIGINT exits and MAVROS
SIGTERM escalation followed confirmed disarm. The full 786-simulation-second
route remains unverified; sampled clearance does not guarantee continuous safety.
Exact commands, resolved limits, speed comparison, evaluation, final state,
source provenance and sampled frames are in this run's
`reports/separated-verification.json`, `speed-comparison.json`, adjacent reports,
telemetry and recording.

The southwest-start revision places UAV 1 and the initial target at `[18,18]`,
and UAV 3 at `[10,14,3]`, 8.94 m southwest of the leader. Headless camera/video
run `20261029T152520Z-17272` completed 300 wall seconds of tracking (87.4 simulation
seconds, 75.8 m of target travel), followed by fleet landing/disarm and normal stop.
UAV 3 first joined its west slot 4.9 simulation seconds after target motion began;
all five first entered TRACK after 20.2 seconds. Minimum sampled separation rose
from 0.786 m in the previous layout to 1.111 m (UAV 1/UAV 5); UAV 3/UAV 4 stayed
at least 2.432 m apart, with no sampled pair below 1 m. Minimum body clearance
was 0.363 m. UAV 1 remained leader with fresh target input in all 901 airborne
controller samples; all 724 sampled airborne leader images contained the marker.
The initial marker centroid was `[79.5,77.5]` in a 160 × 160 image.

UAV 3 briefly returned to SEARCH after joining (30 cycles between stamps 74.78
and 81.98), then rejoined; all final airborne modes were TRACK. There were 18
nominal CPU-cap failures and 35 rejected-scan cycles; 4,312/4,330 nominal calls
succeeded. Compute p95 was 264.2 ms, above the 100 ms target. All 640 retained
captures matched the unchanged upstream pin and adapter. The 128.4 s recording
decoded all 2,568 frames; reviewed joining, closest-approach and tracking frames
show UAV 3 approaching from the southwest and occupying the west slot. The full
route remains unverified, and sampled distances do not guarantee continuous
separation. Exact commands, comparison, evaluation, final disarmed state,
provenance and sampled video frames are in this run's
`reports/separated-verification.json`, adjacent reports, telemetry and recording.

The previous separated-start layout completed 300 wall seconds of headless
camera tracking in `20261029T150000Z-27105`, followed by landing/disarm of all
five UAVs and normal stop. The target covered 75.7 m in 88.3 simulation seconds.
UAV 1 remained leader in all 907 airborne controller samples, with fresh target
input throughout; its marker was visible in all 731 sampled airborne images.
Before target motion, the marker centroid was `[79.5,78.5]` in a 160 × 160 image.
Only UAV 1 supplied detections during the first tracking second. Followers first
joined TRACK 16.6–20.6 simulation seconds after target motion began. UAV 3 briefly
returned to SEARCH during a later bend (35 cycles between stamps 100.88 and
107.78), then rejoined; the final airborne modes were all TRACK.

Minimum sampled body clearance was 0.376 m, and UAV separation was 0.786 m,
above the configured 0.72 m body diameter but below the soft 1 m separation
objective. Of 907 airborne cycles, 842 were ready, 46 rejected tilted scans and
19 had a nominal CPU-cap failure (all on followers); 4,286/4,305 nominal calls
succeeded. Fleet compute p95 was 247.5 ms, above the 100 ms target. All 640 retained
captures matched the unchanged upstream pin and adapter. The 121.2 s recording
decoded all 2,424 frames; reviewed start, approach, formation, tracking and landing
frames retained five complete state/FOV panels. No worker timeouts or Python
tracebacks were observed. Shutdown retained the known SITL SIGINT exits and
MAVROS SIGTERM escalation after confirmed disarm. The complete route remains
unverified. Commands and detailed observations are in this run's
`reports/separated-verification.json`, evaluation, closest-approach report,
final mission state, sampled PNGs, telemetry and video.

The earlier separated-start attempts remain in `20261029T143816Z-18798` and
`20261029T144610Z-7010`. The first exposed initial marker loss and close follower
convergence; the second retained the marker but grazed the configured body margin
at a building on UAV 2's approach. That layout staggers follower starts,
uses 0.6 m/s for the first two target legs before restoring 1 m/s, and starts UAV 2
farther southwest to approach west of that building. Controller/solver behavior,
obstacles, route geometry, sensing, safety margins and expiry are unchanged.

The faster, tighter five-UAV preset raises target speed from 0.6 to 1.0 m/s
and MPC `VMAX` from 1.5 to 2.2 m/s. Its regenerated seed `200205` retains 480
obstacles, with 162 roadside placements, a 2.0 m route-to-disc clearance band,
and 2.114 m minimum sampled route-to-footprint clearance. Acceleration,
separation/body margins, sensing, solver CPU cap and supervision are unchanged.
Headless ground-truth/video run `20261029T123623Z-3955` completed 300 wall
seconds of tracking and landed/disarmed all five UAVs with ownership released.
The target traveled 95.3 m at a measured mean moving speed of 0.999 m/s;
median measured UAV speeds were 0.974–1.013 m/s. Minimum sampled body clearance
was 0.671 m and separation 1.254 m. Of 983 airborne cycles, 975 were ready,
six awaited valid scans and two had UAV 4 CPU-cap failures; 4,883/4,885 nominal
solver calls succeeded. Fleet compute p95 was 179.5 ms, exceeding the 100 ms
target. UAV 4 briefly reached 2.414 m/s: MPC `VMAX` is a prediction bound, and
measured flight can overshoot it. Formation slots contracted to 40% of normal.
All 640 retained captures matched the unchanged upstream pin and adapter.
The 118.35 s video decoded all 2,367 frames; reviewed 45/85 s frames show all
five UAVs passing the tighter buildings/trees with complete state/FOV panels.
Only the first 95 m of the 464 m route was flown. Exact commands, evaluation,
final disarmed state and provenance are in `reports/fast-tight-verification.json`
and adjacent reports; diagnostics are under `artifacts/large200-fast-tight/`.

The same inputs then completed 600 wall seconds of headless camera tracking in
`20261029T124822Z-28878`, followed by fleet landing/disarm and normal stop.
All 1,796 airborne cycles were ready and all 8,980 nominal solves succeeded,
with fresh target input throughout. The target traveled 176.6 m at 1.000 m/s;
median measured UAV speeds were 0.972–1.004 m/s and their maxima ranged from
1.528 to 2.026 m/s. Minimum sampled body clearance was 0.550 m (UAV 3 at
`[69.468,68.455]`), separation was 1.201 m and camera-estimation error p95 was
0.157 m. Slots contracted to 40% of normal. All 640 retained captures matched
the unchanged pin/adapter. Fleet compute p95 remained 226.0 ms, above the
100 ms target; simulation advanced at 0.294× wall time, and the full route
remains unverified. The 204.6 s recording decoded all 4,092 frames, including
the closest approach at 114.2 s. Reviewed tracking/landing frames retain all
five FOV/state panels and show the fleet passing between nearby scenery.
Normal shutdown retains SITL SIGINT exits and MAVROS SIGTERM escalation after
confirmed disarm; no worker timeouts or Python tracebacks were observed.
Commands, per-UAV outcomes/speeds, final state and provenance are in this run's
`reports/fast-tight-verification.json`; `closest-approach.json`, `video.json`,
sampled PNGs, evaluation, telemetry and container logs preserve the evidence.

The five-UAV observer was subsequently restored to a higher angled offset
`[-12,-12,40]` (metres from the target), preserving 3 m UAV flight altitude.
Headless camera/video run `20261029T104431Z-32569` completed 240 wall seconds of
tracking with all 746 airborne fleet cycles ready, all 3,730 nominal solves
successful, and all five UAVs landed/disarmed. The reviewed 101.15 s recording
decoded all 2,023 frames at 1920 × 1080 / 20 FPS and generally shows the fleet
and surrounding city, with occasional partial building occlusion as requested.
All five FOV/state panels fit. Compute p95 was 220.3 ms, above the 100 ms target;
only the initial slalom was flown. Each UAV's retained captures matched the pin
and unchanged adapter. Evidence and exact commands are in the run's
`reports/overhead-verification.json`, evaluation, final mission state and video
reports; build/renderer diagnostics are under `artifacts/large200-high-angle/`.

The dense five-UAV `large200_5uav` preset has 480 obstacles, with 147 placed near
the actual smoothed target route and early slalom bends. Generation protects
every route segment with a 3.2 m obstacle-disc clearance; the default seed's
minimum sampled footprint clearance is 3.421 m. Headless camera/video runs
`20261029T100258Z-21821` and `20261029T101655Z-13125` each completed 300 wall
seconds of tracking, landing and disarming all five UAVs with ownership released.
All 902 / 919 airborne fleet cycles were ready; all 4,510 / 4,595 nominal solves
succeeded. Sampled minimum body clearance was 0.994 / 0.945 m and separation
1.846 / 1.918 m. No flight limits or solver deadlines were relaxed.

The first recording exposed building occlusion in the angled overhead view.
That verification used a vertical 24 m observer offset. Its final video decoded
all 2,324 frames (116.2 s, 1920 × 1080 / 20 FPS); reviewed tracking frames show
all five UAVs between nearby obstacles through the previously obscured bend,
with five complete FOV/state panels. Camera estimation error p95 was 0.080 m.
Fleet compute p95 was 226.4 ms, above the 100 ms target on every airborne cycle,
and simulation ran at 0.296× wall time. No worker timeouts or CPU-cap failures
occurred; every UAV's 128 retained captures matched unchanged upstream pin
`3f71d4c` and the local adapter. Only the initial slalom portion was flown;
the complete route takes 773.95 simulation seconds. Normal stop retains the
known SITL SIGINT exits and MAVROS SIGTERM escalation after disarm.
The final run also logged a marker detector `RuntimeError` in ROS subscription
teardown after SIGINT, following confirmed fleet disarm; it did not interrupt
tracking or video finalization and remains visible in the saved container log.

Commands, provenance, limits, evaluation, final mission state and video evidence
are in each run's `reports/dense-verification.json`, other reports, sampled PNGs,
telemetry and captures. Build/geometry diagnostics are in
`artifacts/large200-dense/`. The initial sparse-field evidence remains in
`20261029T093036Z-28025/reports/large200-verification.json`; its preflight-only
interrupted launch is preserved in `20261029T092510Z-20856`.

The original `small50_3uav` preset also completed a headless camera/video mission
with 30 wall seconds of tracking in `20261029T093839Z-29754`, using its unchanged
fleet, scenario and controller. All 241 evaluated airborne cycles were ready,
all 723 nominal solves succeeded, and the fleet landed/disarmed. Sampled minimum
body clearance was 0.795 m and UAV separation 1.811 m. Compute p95 was 145.3 ms,
with 88.8% of cycles over 100 ms. Its 51.2 s video decoded all 1,024 frames and
the inspected tracking frame retained three complete FOV/state rows. Normal stop
completed; logs retain the known SITL SIGINT exits and MAVROS SIGTERM escalation
after disarm. Evidence is in that run's `reports/` and
`artifacts/large200-5uav/small-*.log`.

The simulation-only refactor was rebuilt and checked with two headless,
GPU-rendered missions, each with 30 wall seconds of tracking and video enabled.
Development camera run `20261029T083900Z-12828` completed with all UAVs
landed/disarmed, 237/239 fleet-ready airborne cycles, 0.798 m minimum sampled
body clearance and 1.761 m separation. Its finalized 1920 × 1080 / 20 FPS video
contains 1,010 frames (50.5 s), including takeoff, tracking and landing.

Standalone installed-image run `installed-20261029T084146Z` used read-only root
and configuration mounts, no source checkout or Git metadata, and bundled
upstream sources with ground-truth target input. All UAVs landed/disarmed;
238/240 airborne cycles were fleet-ready, with 0.819 m minimum sampled body
clearance and 1.827 m separation. Its finalized video contains 939 frames
(46.95 s) at the same resolution/rate. Both videos decoded completely and their
sampled frames showed the overhead view, growing sensor map, state instruments,
UAV camera feeds and landing. Server Ogre logs confirmed NVIDIA RTX 5050
acceleration. The GUI/display configurations resolved; an interactive GUI session
and the full route were outside these checks.

Every UAV's retained optimizer captures matched the unchanged upstream pin
`3f71d4c` and local adapter hash in both runs. UAV 1 reached the unchanged CPU
cap twice per run; other UAV solves succeeded. Fleet compute p95 was 149.3 ms
(camera) and 132.5 ms (installed), exceeding the 100 ms target. Normal cleanup
finalized both videos; logs retain SITL SIGINT exits and pinned MAVROS SIGTERM
escalation after disarm. Evidence is in each run's `reports/evaluation.json`,
`simulation-only-refactor.json`, `mission-final.json`, `video.json`, contact sheet,
telemetry and logs. Build/installed-runtime diagnostics are under
`artifacts/simulation-only-refactor/`. Historical experiments remain in their
saved run directories and Git history.

Local speed recovery was enabled and checked in headless ground-truth run
`20261029T090233Z-23158`, with 30 wall seconds of tracking, no video, and a
run-only capture limit of 512. All 328 captured solves per UAV succeeded against
unchanged upstream pin `3f71d4c` with the original 0.2 s CPU cap. UAV 1 had seven
overspeed input calls (maximum 1.444 m/s); recovery permits transient overspeed
and does not remove flight tracking error. All 306 evaluated airborne cycles
were ready, with 0.855 m minimum sampled body clearance and 1.786 m separation.
Fleet compute p95 was 122.1 ms; 53.9% of cycles exceeded 100 ms. This single run
does not establish timing improvement over the video-enabled historical runs;
simulation advanced at approximately real time here. All UAVs landed/disarmed
before normal stop, with no worker timeouts or Python tracebacks. Shutdown
retained the known SITL SIGINT exits and MAVROS SIGTERM escalation. Evidence and
exact commands are in the run's `reports/speed-recovery.json`, `evaluation.json`,
`mission-final.json`, optimizer captures, telemetry and logs. Camera perception
and the full route were outside this check.
