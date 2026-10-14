# Validation record

[Back to the project overview](../README.md)

This document consolidates recorded simulation validation from **9–14 October
2026**. Each entry identifies its configuration and scope. The 14 October entry
covers the parallel execution change with a short flight. Generated evidence in
`runs/` and `artifacts/` is Git-ignored and may be absent from a fresh clone.

## Test environment

The recorded host ran Ubuntu WSL2 with Docker Desktop 29.8.2, Compose 5.5.1,
Linux amd64, 14 allocated CPUs, and about 15.37 GiB RAM. WSLg display access
worked, but the inspected renderer was `llvmpipe` with no hardware acceleration.
Runtime components were ROS 2 Jazzy, ArduCopter 4.5.3, MAVROS source overlay
2.15.1, Gazebo Harmonic / gz-sim 8.15.0, ROS Python 3.12.3, and Waf Python 3.11.16.
Exact source pins are in [versions.json](../locks/versions.json); APT patch versions
can change on rebuild.

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
