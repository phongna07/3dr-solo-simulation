# MPC migration: small50_3uav in Gazebo

Three Iris UAVs hold approximately 3 m altitude in the original 50 × 10 m field.
Rectangles and circles become solid 5 m boxes and cylinders. A magenta ground
marker follows the target spline; downward cameras detect it, while horizontal
LiDARs measure obstacle surfaces. The original motion_planning_MPC folder is not
modified and is not needed at runtime. The controller remains planar.

## Run step by step

```bash
./scripts/dev build sim
./scripts/dev test
./scripts/dev mpc
./scripts/dev mission status
./scripts/dev gui                       # optional, separate terminal
./scripts/dev mission run               # takeoff, target tracking, LAND
./scripts/dev evaluate
./scripts/dev stop
```

Bringup never arms vehicles. `mission run` is simulation-only and enables the
publisher after every UAV takes off. Interrupting it attempts to pause the target,
release publishing and LAND. `mission run --seconds 20` limits a test to 20 wall
seconds after takeoff. `mission start` starts tracking and returns; use `mission
stop` to finish. Do not publish old position/velocity flight commands while MPC
is enabled. Normal `dev stop` releases MPC before LAND; `stop --force` immediately
stops processes and does not land.

For a controller baseline with target ground truth:

```bash
MPC_TARGET_MODE=ground_truth ./scripts/dev mpc
./scripts/dev mission run --seconds 20
```

Restart between experiments. Camera mode is the default and has no MPC
subscription to target ground truth. Truth is used for initial disarmed frame
alignment and evaluation. The mission supervisor uses target ground truth to
identify route completion. LiDAR measurements always come from Gazebo sensors.

## Modify the algorithm

The package is `ros_ws/src/solo_mpc`. Python modules are bind-mounted and take
precedence over installed modules. Change Python/config and restart the mission.
Adding executables or dependencies requires rebuilding the image.

| Module in solo_mpc | Responsibility |
|---|---|
| core/mpc_problem.py | Original CasADi/IPOPT costs and constraints |
| core/dynamics.py | Original acceleration-lag model |
| core/planner_grid.py, planner_jps.py | Original local Dijkstra/grid helpers |
| core/controller.py | Measured-state planning, committed paths, sticky leader, Hungarian slots, corridor construction |
| core/mapping.py | Explicit ray validity, clearing of old occupancy |
| core/tracking.py | Timestamped target filter and bounded extrapolation |
| core/sensors.py | Scan projection, camera geometry, pose history |
| controller_node.py | ROS input/output and measured-state logging |
| setpoint_node.py | Independent publishing, expiry, altitude, proximity guard |
| frame_node.py | Disarmed translation alignment and measured world state |
| perception_node.py | Actual camera image detection of the magenta marker |
| target_node.py | Gazebo-clock target motion and pause/reset |
| world.py | SDF field, target and sensors |

`TeamController.step(states, scans, stamp, seers)` has no ROS imports and never
integrates measured vehicle state. Configuration is explicit, without scenario
imports or class-level team state. The original solver, discretization and planner
are retained; simulator-specific orchestration is replaced by an instance-local
coordinator. Source provenance and MIT license are in UPSTREAM.json and LICENSE-MPC.
Each controller startup records its source hashes and resolved configuration in
controller_sources.jsonl; per-UAV solver statuses and timings are in mpc.jsonl.

Edit `config/controller.yaml` for deployment settings and MPC overrides; edit
`scenarios/small50_3uav.yaml` for field geometry, starts and waypoints. The overrides
increase the drone radius to 0.36 m, minimum separation to 1 m, and obstacle margin
to 0.15 m. Initial speed/acceleration limits are 1.2 m/s and 2 m/s². Some original
narrow passages become unavailable. Camera defaults are 160 × 160 at 8 Hz with an
approximately 3 m square footprint at flight altitude. No gimbal/video encoder is
used. Identify actual vehicle response before increasing motion limits.

## ROS interfaces

| Topic/service | Type and purpose |
|---|---|
| /uavN/mavros/local_position/pose, velocity_local | FCU measured local state |
| /uavN/mpc/pose, velocity | Measured state in common world ENU |
| /uavN/scan | sensor_msgs/LaserScan, Gazebo returns |
| /uavN/camera/image_raw, camera_info | Camera images and calibration |
| /uavN/target/observation | PoseStamped, camera observation in world |
| /uavN/mpc/reference | PositionTarget with world ENU values |
| /uavN/mpc/prediction | nav_msgs/Path, visualization |
| /uavN/mavros/setpoint_raw/local | Local ENU position/velocity/acceleration; MAVROS converts to NED |
| /mpc/status, /mpc/setpoint_status | JSON std_msgs/String diagnostics |
| /mpc/enable | SetBool, flight publisher ownership |
| /target/run, /target/reset | SetBool pause/resume and Trigger reset |
| /target/ground_truth, /uavN/ground_truth/pose | Simulation alignment/evaluation |
| /clock | Gazebo simulation time |

MPC, perception, frames, target and publisher use simulation time. MAVROS keeps
its validated wall/FCU policy. The adapter maps FCU measurements to simulation
receipt time; sensor projection chooses a nearby measured pose within a configured
tolerance. This approximation introduces latency error and is not exposure-time
FCU synchronization. The pinned Gazebo XYZ-to-NED transform and MAVROS NED-to-ENU conversion give
matching ENU position axes. Their yaw is configured explicitly; compass bias must
not rotate the position frame. Ground truth calibrates only translation while
disarmed; subsequent motion and attitude come from MAVROS. Real hardware needs
independently established common localization and clock synchronization.

## Failure behavior and measurements

Missing scan rays remain unknown. Observed no-return rays clear space. Teammate
endpoints are not accumulated as permanent obstacles. Strongly tilted rays are
discarded rather than flattened. Corridor failures produce no fresh command;
failed decomposition is never treated as obstacle-free space. The publisher
rejects stale/invalid commands and sends zero horizontal velocity with altitude
feedback when measured pose is fresh. It runs in another process so solver stalls
do not block publishing. It is disabled during takeoff and LAND.
Pose freshness and command expiry have strict independent wall limits. Low-rate
FCU status has separate 2.5 s simulation / 10 s wall limits because slow SITL
stretches its heartbeat interval. Loss of fresh measured pose still brakes within
3 wall seconds. Bringup keeps Gazebo and MAVROS running if an MPC process exits,
so the supervised mission can attempt LAND.

Only camera observations renew target validity. On occlusion the filter briefly
coasts, then SEARCH approaches the last plausible location. `mission run` pauses
the target on prolonged loss while the UAVs reacquire it; failure to reacquire
within the search timeout aborts and attempts LAND. This explicit experiment
policy does not model an uncooperative target. `mission start` leaves supervision
to the caller. Original UAV separation/visibility constraints are soft; the
proximity guard adds protection but does not establish a collision guarantee.
A body-mounted 2D scan is not a full 3D obstacle map.

Each run saves generated SDF, input snapshots, optimizer/bridge configuration,
alignment and mpc.jsonl measured-state records. `mission run` also records
truth.jsonl from Gazebo poses for independent geometric evaluation. `dev evaluate` writes evaluation.json
with obstacle body clearance, minimum separation, target estimation error,
tracking distance, simulation speed and compute percentiles. Check
physics contacts, image tracking and renderer performance alongside sampled
metrics. Python dependencies are pinned with hashes in locks/mpc-py312.lock.

GPU acceleration remains host-dependent. Lighter sensors reduce work, but actual
real-time factor must be measured. See WSL_GPU.md for renderer configuration;
increasing timeouts does not establish real-time performance.

## Validation on this host

The camera-driven forward route reached the final 47 m target waypoint and all
three UAVs landed and disarmed. This run used the initial 32-face solver allocation;
Gazebo samples gave 0.361 m minimum body clearance and 1.135 m minimum UAV
separation. Camera target estimation error was 0.094 m median and 0.311 m at the
95th percentile. The target observation model was the actual image pipeline.

The software-rendered simulation advanced at about 0.17 simulated seconds per
wall second. Whole-fleet compute was 156 ms median and 199 ms at the 95th
percentile. Only 59% of cycles had successful solutions for every UAV; failed
vehicles used the expiry/braking behavior and retried. These results establish
functional simulation integration, not real-time hardware readiness. The final
configuration uses the upstream 16-face initial allocation, with automatic growth
when a measured corridor needs more faces. See MPC_VALIDATION.md for final
configuration checks and saved run paths.
