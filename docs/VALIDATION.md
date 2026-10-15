# Validation record

[Back to the project overview](../README.md)

This document consolidates recorded simulation validation from **9–15 October
2026**. Each entry identifies its configuration and scope. The 14 October entry
covers the parallel execution change; the 15 October entry checks persistent
mission supervision after the CLI exits. Generated evidence in
`runs/` and `artifacts/` is Git-ignored and may be absent from a fresh clone.

## Test environment

The recorded host ran Ubuntu WSL2 with Docker Desktop 29.8.2, Compose 5.5.1,
Linux amd64, 14 allocated CPUs, and about 15.37 GiB RAM. WSLg display access
worked, but the inspected renderer was `llvmpipe` with no hardware acceleration.
Runtime components were ROS 2 Jazzy, ArduCopter 4.5.3, MAVROS source overlay
2.15.1, Gazebo Harmonic / gz-sim 8.15.0, ROS Python 3.12.3, and Waf Python 3.11.16.
Exact source pins are in [versions.json](../locks/versions.json); APT patch versions
can change on rebuild.

## Release packaging — 15 October

Native AMD64 images were rebuilt from the pinned source commits with separate
compiler/runtime stages. Runtime image sizes (`docker image inspect .Size`,
decimal GB) changed as follows:

| Image | Previous | Release runtime |
|---|---:|---:|
| Simulation | 8.04 GB | 4.25 GB |
| Companion | 3.33 GB | 2.36 GB |

Local tested image IDs (these are not registry pull addresses):

- Simulation: `sha256:b69386064a5e9ab3031df48d9a211251db62dd783bf4f2fc9a57cb0475496733`
- Companion: `sha256:b84608000fa4c8ca5b36cb03872ffe11b0585d78c7ab4b20396ccaf49bdd62b5`

All **204 tests passed** in the final simulation runtime. The separate
`release-smoke` acceptance test extracted deployment files from each image into
a temporary directory with no checkout. It verified installed module imports,
MAVROS overlay resolution, application/dataset checksums, required shared
libraries, absence of source/build environments and compiler commands, and writes
as UID/GID 1000. The root filesystem and configuration were read-only; only
output binds and temporary storage were writable.

Simulation reached three-FCU telemetry/pose readiness and initialized every node
in **253 wall seconds**, without arming. Camera and LiDAR messages were sampled.
The dedicated, live Gazebo server Ogre log reported Mesa 25.2.8 with
`llvmpipe (LLVM 20.1.2, 256 bits)`, at approximately 0.14–0.19 of real time.
The detector handles Gazebo's rewritten process title as well as ordinary argv;
it does not certify logs from stopped servers or a separate GUI process.

The companion smoke checks ROS/MAVROS imports, launch resolution and filesystem
permissions; it does not connect to an FCU or qualify hardware flight. A companion
replay from its preserved build-input image also built successfully with build
commands' network access disabled and passed the same smoke checks.

Evidence: `artifacts/release-final-tests.log`,
`artifacts/simulation-release-final-smoke/` (readiness, server renderer, sensor
messages, image inspection and process logs), `artifacts/companion-release-final-smoke/`,
and `artifacts/companion-offline-smoke/`. Build inputs are retained separately in
`solo-simulation-inputs:release`, `solo-companion-inputs:release`, and
`solo-base:inputs`; they are not runtime image ancestors.

**ARM64 runtime testing, hardware GPU validation, and registry publication were
not performed in this local validation.** The manual `release-images` workflow
builds and tests on separate native AMD64/ARM64 machines, then publishes passing
platform-specific images with runtime/input/base digests and acceptance evidence.
See [Reproducibility](REPRODUCIBILITY.md) for deployment and preservation instructions.

## Persistent mission supervision — 15 October

Run: `runs/20261015T065414Z-9371`, using the default three-UAV camera scenario,
parallel controller, and 3 m flight altitude. The rebuilt local image ID was
`sha256:ccd3426614a2fb3d8666f55fc0719174553168e5bebd155b0b1217c22aaf515e`,
with live project sources mounted. Initialization took 263 wall seconds under
software rendering, at approximately 0.13–0.19 simulation seconds per wall second.

`./scripts/dev mission start --seconds 10` returned successfully during tracking.
Subsequent independent status requests observed `tracking`, `landing`, then
`completed` with no mission error. The persistent supervisor enforced the
duration, disabled publishing, paused the target, and landed/disarmed all three
UAVs after the CLI had exited. Its `truth.jsonl` contains 938 samples spanning
simulation time 21.15–68.0 s; maximum vehicle altitudes were approximately 3.12 m.
Normal `dev stop` then completed successfully.

`./scripts/dev test` passed all 165 tests. Separate ROS process tests verify
supervision after CLI exit, duration enforcement, and cancellation during remote
takeoff with a paused simulation clock. Policy tests exercise target loss and
reacquisition, prolonged planner failure, arrival and arrival timeout, stale
inputs, flight-envelope violations, request serialization, and landing failures.
Those injected failures use simulated flight I/O; the Gazebo flight above checks
takeoff, short tracking, and automatic landing rather than full-route completion.

Evidence: `artifacts/mission-supervisor-validation.json`,
`mission-supervisor-flight-status.jsonl`, `mission-supervisor-simulation.log`,
`mission-supervisor-tests.log`, and `mission-supervisor-stop.log`.

## Complete camera-driven route

Run: `runs/20261011T055529Z-29926`. Evidence: the run's `evaluation.json`,
`artifacts/mpc-camera-flight.log`, and `artifacts/mpc-camera-evaluation.json`.

Three UAVs took off to approximately 3 m, tracked the moving magenta marker
through the obstacle field using camera detections, reached the final target
waypoint near `(47, 5.5)`, and landed/disarmed. LiDAR input came from Gazebo sensors.
The camera-mode controller did not subscribe to target ground truth; evaluation
used independent Gazebo vehicle poses.

| Metric | Recorded result |
|---|---:|
| Airborne controller samples | 747 |
| Minimum sampled obstacle body clearance, including 0.36 m radius | 0.361 m |
| Minimum sampled inter-UAV center distance | 1.135 m |
| Nearest UAV-to-target distance, median / p95 / maximum | 1.080 / 1.553 / 1.829 m |
| Target estimate error, median / p95 / maximum | 0.094 / 0.311 / 0.922 m |
| Whole-fleet compute, median / p95 / maximum | 156 / 199 / 270 ms |
| Cycles with successful solutions for all three UAVs | 58.8% |
| Simulation seconds per wall second | 0.172 |

This route used the initial **32-face** solver allocation. During disarmed
bringup, the controller was restarted to apply corrected ENU alignment and
compiled scan integration; the forward route used those corrections. Its initial
source manifest predates that restart. Current controllers additionally record
startup hashes/settings in `controller_sources.jsonl`.

These sampled clearances do not establish continuous collision avoidance.
Separation constraints are soft, and the independent publisher expires references
and brakes after failed solves. Compute time exceeded the 100 ms target period;
this run establishes functional simulation integration, not real-time readiness.

## Checks of the 16-face allocation and heartbeat fix

The current initial corridor allocation is 16 faces, growing when a measured
corridor requires more. The recorded follow-up checks were:

| Check | Evidence and outcome |
|---|---|
| Pure-controller forward route | `artifacts/mpc-core-route-16.log`: 850 steps, 0.402 m minimum body clearance, 1.145 m minimum separation, 55 failed individual solves out of 2,550 attempts |
| Gazebo stationary endpoint flight | `runs/mpc-endpoint-verification`, `artifacts/mpc-endpoint-flight.log`: 22 wall seconds of publishing, then LAND/disarm; 43 airborne cycles, 90.7% whole-fleet successful cycles, 127 ms median / 157 ms p95 compute, 0.030 m median target estimate error |

The pure-controller route used synthetic scans and the algorithm's dynamics,
not Gazebo physics or camera images. The stationary endpoint is a different
workload from the moving obstacle route, so its metrics are not a direct speed
comparison.

The endpoint run exposed false FCU-staleness braking under slow SITL. FCU status
now has separate 2.5 s simulation / 10 s wall limits; pose freshness remains 3 wall
seconds and command expiry remains 0.4 simulation / 3 wall seconds. ROS regression
tests verified this distinction. **A complete Gazebo forward route with the
16-face configuration and final heartbeat correction has not been rerun.**

## Earlier bringup and automated checks

The 9 October baseline verified isolated FCU links and SITL state, telemetry
IDs 30/32, finite local pose, prefixed frames, takeoff/velocity/position control,
and LAND/disarm for three UAVs. A sensor-light control profile achieved about
9.94 Hz wall pose rates and real-time factor 1.0. The original camera/gimbal
reference profile ran around 0.23 real-time factor with readiness dropouts under
render load. These historical profiles are not the current mission CLI workflow.

The 10 October regression fixed the pinned MAVROS parameter-callback crash and
host waiting on an exited container. Bootstrap reads current parameters and
writes only differences; unsupported attitude-mode changes are rejected. Fresh
three-UAV readiness was observed after the fix. The mechanism is documented in
[Development](DEVELOPMENT.md#mavros-parameter-bootstrap).

On 11 October, all **37 tests in the then-current suite** passed inside the
simulation image. They covered controller/state ownership, scans and mapping,
camera projection, target filtering, frame/setpoint conversion, command and FCU
freshness, ROS construction, and bringup configuration. Three warnings came from
Gazebo/protobuf dependencies. Evidence: `artifacts/mpc-final-image-tests.log`.
Generated SDF validation, shell syntax, Compose parsing, and whitespace checks
also passed, and sensor/truth/observation streams were observed in ROS.

Simulation and companion amd64 images were built. The MPC validation build used
`artifacts/Dockerfile.mpc-validation` with host-downloaded, hash-checked Python and
ROS Debian packages because container downloads stalled. Application/base/compiler
stages matched the canonical Dockerfile; two dependency install steps differed.
Evidence: `artifacts/mpc-build-final.log`. The normal build remains
`./scripts/dev build sim`; the temporary caches are not runtime dependencies.

## Parallel execution check, 14 October

The default now uses one persistent spawned mapping/planning/corridor/MPC worker
per UAV and one camera detector process per UAV. The ten-step horizon, 16-face
initial allocation, solver settings and two-neighbor avoidance constraints are
retained. Formation checks use all workers' current maps, and all solves receive
the same frozen neighbor snapshot. This is local multiprocessing; physical
three-Pi networking and neural detection remain unimplemented.

All **67 tests passed** inside `solo-simulation:gd0` (three dependency warnings).
New regression coverage checks real solve overlap, command/trajectory parity with
the sequential reference, persistent occupancy maps, measured-state ownership,
single-UAV operation, stalled/disconnected workers, shutdown and camera isolation.
Python syntax, Compose configuration and whitespace checks also passed.

Headless run `runs/20261014T044239Z-15013` used the default three-UAV camera
scenario with `gz sim --headless-rendering -s -r`; no GUI was opened. The fleet
tracked the target, then landed and disarmed. The simulator was stopped afterward.
A full Gazebo forward route and physical Pi/neural-detector validation remain
outstanding.

## Remaining validation

- Validate hardware GPU rendering and achieved sensor/control rates. The recorded
  mission workloads ran at approximately 0.17–0.19 real-time factor in software.
- Confirm the 100 ms deadline on physical Pis with the selected neural detector.
  Measure tail latency and publisher jitter under the complete hardware load.
- Rerun the full moving-target route with the current defaults and record source,
  input snapshots, image identity, solver failures, and landing outcome.
- Validate any new fleet size independently. The planner remains fixed-altitude
  and planar; 2D scans do not represent arbitrary vertical geometry.
- Run remote CI and physical ARM64/FCU/sensor validation. Pi performance, Solo
  firmware compatibility, LD19 integration, concurrent GoPro recording/streaming,
  common localization, and real-flight failsafes remain unverified.

Use [Operations](OPERATIONS.md#logs-and-evaluation) to collect current evidence and
[Hardware migration](HARDWARE.md) for the staged physical deployment plan.
