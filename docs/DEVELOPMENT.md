# Development

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Configuration](CONFIGURATION.md) · [Deployment](DEPLOYMENT.md)

Read [AGENTS.md](../AGENTS.md) before editing. Keep numerical code independent
of ROS and preserve measured-state control and independent flight supervision.

## Architecture

`scripts/dev` starts Docker Compose; `solo_mpc/launch/mpc.launch.py` owns Gazebo, one SITL/MAVROS
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
