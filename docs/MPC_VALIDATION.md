# MPC migration validation — 2026-10-11

The implementation was exercised with ROS 2 Jazzy, Gazebo Harmonic,
ArduCopter 4.5.3 and MAVROS 2.15.1 in Docker Desktop / WSL. This host has no
usable GPU adapter in Docker; rendering uses software. The original scenario
and algorithm sources remain in `/home/phong/motion_planning_MPC` unchanged by
this migration.

## Complete camera-driven Gazebo route

Run directory: `runs/20261011T055529Z-29926`.
Mission log: `artifacts/mpc-camera-flight.log`.
Metrics: the run's `evaluation.json` and `artifacts/mpc-camera-evaluation.json`.

Three UAVs took off to approximately 3 m, followed actual camera detections of
the moving magenta marker through the extruded small50 field, reached the final
target waypoint at approximately (47, 5.5), then landed and disarmed. LiDAR input
was actual Gazebo LaserScan data. Target truth was not subscribed to by the
camera-mode MPC. Evaluation used independent Gazebo vehicle poses.

| Metric | Result |
|---|---:|
| Airborne evaluation samples | 747 |
| Minimum sampled obstacle clearance, including 0.36 m body radius | 0.361 m |
| Minimum sampled inter-UAV center distance | 1.135 m |
| Nearest UAV to target, median / p95 / maximum | 1.080 / 1.553 / 1.829 m |
| Camera target estimate error, median / p95 / maximum | 0.094 / 0.311 / 0.922 m |
| Whole-fleet compute, median / p95 / maximum | 156 / 199 / 270 ms |
| Cycles with successful solutions for all three UAVs | 58.8% |
| Simulation seconds per wall second | 0.172 |

This experiment used the initial 32-face solver allocation. During initial
disarmed bringup the controller was restarted to apply the corrected ENU
alignment and compiled scan integration. The complete forward route ran with
those corrections. The run's initial source manifest predates that restart;
current controllers additionally record startup source hashes and resolved
settings in `controller_sources.jsonl`.

Sampled clearances do not establish continuous collision avoidance, and this
experiment does not validate operation on physical UAVs. Original separation
constraints are soft. Failed solves produce no fresh reference; the independent
publisher expires commands and brakes. Frequent solver retries and the measured
compute budget remain limitations.

## Final solver allocation

The final default starts with 16 corridor faces, matching the upstream algorithm,
and grows when a measured corridor needs more. Two additional checks used it:

1. The pure-controller model followed the complete forward route for 850 steps.
   Minimum body clearance was 0.402 m and minimum separation 1.145 m. There were
   55 failed individual solves out of 2,550 attempts. This uses the algorithm's
   dynamics and synthetic scans, rather than Gazebo physics or camera images.
   Log: `artifacts/mpc-core-route-16.log`.
2. An actual three-UAV Gazebo flight at the final marker waypoint verified the
   reduced allocation, measured-state control and camera acquisition. The
   marker was paused, the publisher was enabled for 22 wall seconds, and all
   three UAVs subsequently landed and disarmed. Run directory:
   `runs/mpc-endpoint-verification`; flight log:
   `artifacts/mpc-endpoint-flight.log`. There were 43 airborne evaluation cycles;
   90.7% had successful solutions for the whole fleet. Whole-fleet compute was
   127 ms median and 157 ms p95. Per-UAV diagnostics recorded 125 successful
   solves and four CPU-limit failures. The camera estimate error was 0.030 m
   median. These stationary-endpoint numbers are not a direct performance
   comparison with the obstacle route.

The endpoint publisher diagnostics also revealed false FCU-staleness braking:
slow SITL stretches low-rate status messages beyond the original shared 3 s wall
limit. The final publisher has separate FCU limits of 2.5 simulation seconds and
10 wall seconds, while pose freshness remains 3 wall seconds and command expiry
remains 0.4 simulation seconds / 3 wall seconds. ROS message-level regression
tests verify this distinction and braking on actual stale status. The final
heartbeat-limit correction was tested at the ROS adapter level; a complete
Gazebo forward route with the final 16-face/default settings has not been rerun.

## Automated and build checks

`./scripts/dev test` runs the suite inside the simulation image. All 37 tests
passed, including real ROS node construction, ENU setpoint transforms, stale
command braking, low-rate/stale FCU status, measured-state ownership, ray
validity, occupancy clearing, teammate filtering, camera projection, target
coasting, world geometry and existing bringup configuration. Three warnings
come from the installed Gazebo/protobuf Python dependencies.

The generated SDF passed `gz sdf -k`. Shell syntax, Compose configuration and
`git diff --check` passed. All sensor, truth and target-observation streams were
observed in the running ROS graph. Camera snapshots are saved under `artifacts/`.

The `solo-simulation:gd0` image was built with the pinned numerical dependency
lock. Container network downloads stalled on this host, so the validation build
used a temporary Dockerfile under `artifacts/` with host-downloaded, hash-checked
Python packages and ROS Debian packages as BuildKit contexts. The application,
base-image and compiled dependency stages are the same as the canonical
`docker/Dockerfile`; only the two dependency download/install steps differ.
Normal builds remain `./scripts/dev build sim`.

Validation build log: `artifacts/mpc-build-final.log`. Final suite log:
`artifacts/mpc-final-image-tests.log`. The temporary wheel/deb caches and generated
run data are Git-ignored and are not runtime dependencies. Share the built image
or use the normal network build to reproduce on another machine.

## Remaining bottlenecks

- Software rendering limits the simulator to roughly 0.17–0.19 real-time factor
  with the three cameras and LiDARs. Validate GPU access before increasing
  camera resolution or sensor rates.
- Serial whole-fleet MPC computation still exceeds the 100 ms target period.
  Profile the planner, corridor construction and per-UAV solves before changing
  horizon, solver limits or moving solves to separate workers.
- The fixed-altitude planner handles horizontal obstacle footprints. It cannot
  fly over obstacles or represent arbitrary vertical geometry with its 2D scans.
- The marker detector assumes a known flat target plane. Hardware deployment
  needs a suitable detector, common localization, calibrated extrinsics,
  exposure-time synchronization and measured vehicle dynamics.

Follow `MPC_MIGRATION.md` for launch, mission control, configuration and module
boundaries.
