# Simulation

[Overview and quick start](../README.md) · [Configuration](CONFIGURATION.md) · [Development](DEVELOPMENT.md)

Run commands from the project root. Initialize the submodule before building:
`git submodule update --init --recursive`. Keep your existing `.env`; create it
from `.env.example` only for a new checkout.

## Run a mission

```bash
./scripts/dev build sim
./scripts/dev doctor
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

`mpc` starts a fresh stack and waits for processes, controller nodes, FCU
connections, telemetry acknowledgements, and local poses. It does not arm.
Takeoff waits for aligned frames, fresh inputs, and a stable fleet altitude
before enabling target motion and control. Startup can take several minutes
with slow rendering. `--seconds` limits tracking in wall time, excluding
startup/takeoff/landing; omit it to attempt the complete route.

For the five-UAV 200 × 200 m random field, use the matching preset:

```bash
./scripts/dev mpc --preset large200_5uav --record-video
./scripts/dev mission run --seconds 300
./scripts/dev evaluate
./scripts/dev stop
```

To watch distant followers rendezvous with a leader already over the target,
select `--preset large200_5uav_separated` in the same commands. Keep camera mode
(the default) to check that UAV 1 retains the pink target while the other four
approach from about 9–37 m away. UAV 3 starts southwest of the leader to reduce
crossing during joining. Its UAV speed bound is 1.2 m/s; the target uses 0.4 m/s
on its first two legs, then 0.6 m/s. Allow at least 450 wall seconds for joining
and subsequent tracking, and inspect the recorded modes and measured separations; wall time
is longer than simulation time on a loaded renderer.

The original fast large preset takes about 464 simulation seconds (7.7 minutes
at real time) to complete its route, with several early slalom bends through
nearby obstacles. The slower separated preset takes about 786 simulation seconds
(13.1 minutes at real time). Use 450 wall seconds or more to inspect joining
and multiple bends when rendering slows the clock.
A time-limited run covers only part of it; rendering five sensors and a large
obstacle field can make simulation slower than wall time. Use the observed clock
rate when choosing a verification duration. Select `--preset small50_3uav` to
return to the original configuration; see [Configuration](CONFIGURATION.md#five-uavs-in-a-200--200-m-map)
for regeneration and parameter details.

| Command | Purpose |
|---|---|
| `mpc --ground-truth-target` | Exact timestamped target input; omit detectors/cameras unless recording |
| `mpc --record-video` | Record overhead, map, state instruments, and all UAV FOVs |
| `mpc --gui` / `gui` | Attach Gazebo viewer; closing it leaves simulation running |
| `mpc --no-wait` / `wait 600` | Start asynchronously / wait for readiness |
| `mission start --seconds 30` | Return after tracking starts; supervisor continues |
| `mission status` | Mission state, ownership, errors, and controller diagnostics |
| `mission stop` | Pause target, disable publishing, LAND eligible UAVs, await disarm |
| `mission reset` | Reset the target while idle |
| `status` / `logs` / `shell` | Health / process logs / container shell |

Prefix the commands above with `./scripts/dev`. Only one simulation should run
at a time. FCUs and ROS nodes share one container; host ROS discovery and exposed
FCU ports are unnecessary. Use `shell` to inspect topics.

## Stop and recover

Ctrl+C in `mission run` requests cleanup. Confirm landing and disarm before
restarting. The supervisor remains responsible after a `mission start` client
exits. Target loss, stale inputs, persistent controller failures, or flight
violations trigger cleanup.

Normal `./scripts/dev stop` requests mission cleanup before stopping the container.
If landing fails, inspect FCU state and service errors and retry `mission stop`.
Use `stop --force` only for a crashed simulation when normal landing cannot
complete. A failed command in an `&&` chain skips later cleanup commands.

Ownership is per UAV. A pilot mode change or disarm releases that UAV, stops
publishing, and lands remaining mission-owned UAVs; cleanup does not send LAND
to released or unowned aircraft. New missions require a connected, disarmed fleet.
A timed-out worker is disabled until the controller restarts; extending the
tracking duration cannot repair it. Preserve the failed run before restarting.

## Evidence and evaluation

Use the actual `runs/<run-id>/` directory, not a changing `runs/latest` link.

| Location | Evidence |
|---|---|
| `config/` | Fleet/scenario/controller snapshots, resolved `mpc-config.json`, world and bridge files |
| `metadata/` | Manifest, controller and upstream source hashes, dependency fingerprints |
| `status/` | Startup, live health, readiness, alignment, and node initialization |
| `telemetry/mpc.jsonl` | Measured state, solver results, worker timing, controller decisions |
| `telemetry/truth.jsonl` | Independent Gazebo poses and target positions |
| `optimizer-inputs/<uav>/` | Bounded complete solver calls for numerical replay |
| `reports/evaluation.json` | Sampled geometry, tracking, compute, and simulation speed |
| `logs/`, `sitl/`, `gazebo-home/` | ROS/process logs, FCU state, renderer diagnostics |
| `recorded-mission.mp4` | Optional combined video |

The image ID is saved beside the run as `runs/<run-id>.image-id.json`.
Evaluate after landing, before stopping the container:

```bash
./scripts/dev evaluate --run-dir /workspace/runs/YOUR_RUN_ID
```

Evaluation prefers Gazebo geometry and reports its source. Its airborne filter
requires every UAV above 2.5 m; revisit that filter when changing altitude.
Inspect solver failures and measured motion alongside the summary. Sampled
clearance and successful commands alone do not establish continuous safety.

## Recording

Select `--record-video` at launch, optionally with `--ground-truth-target`.
Normal `stop` finalizes `recorded-mission.mp4`; mission completion and GUI closure
do not. Recording starts at takeoff and continues through landing.

The 1920 × 1080 / 20 FPS output combines a target-following overhead view,
sensor-observed occupancy map, state instruments, and a FOV panel per UAV.
The right-hand square panels shrink and stack vertically to fit every UAV;
the left map/state column gives additional height to larger fleets so all five
status rows remain visible. Labels and fleet colors follow the configured order.
Colored coverage rays/outlines appear in the overhead view and GUI, but are
excluded from UAV images and LiDAR. They show flat-ground camera projections;
obstacles can occlude the view. Distant UAVs can be outside the overhead frame.

State instruments use world ENU position (m), velocity (m/s), estimated and
commanded acceleration (m/s²), and measured yaw. Acceleration estimates are
smoothed velocity derivatives, excluding gravity. Commands precede publisher
guards. Arrow scales are 2 m/s or 2 m/s² at the rim. Missing/stale values show
`--`. Map cells show occupied/free/unknown sensor evidence, not obstacle truth.

Feeds align by original simulation timestamps; slower feeds hold their last
frame. `telemetry/recorded-mission.timestamps.jsonl` maps output frames to time.
Each combined-frame record also includes `source_stamps_ns` for the overhead,
UAV images, and occupancy map; repeated source stamps identify held feeds.
The Full HD overhead bridge and recorder use reliable ROS delivery with a
20-image history so lost DDS fragments can be retransmitted. Best-effort
delivery of these large images can cause periodic freezes even when Gazebo
renders at 20 FPS; the smaller perception feeds retain sensor-data QoS.
Restart the stack after changing recording code or bridge configuration.
Rendering/encoding adds load. Add rosbag recording from `shell` only when needed,
using a fresh output directory and stopping it before the container.

## GPU and display

Rendering sensors need a working server renderer even without a GUI. Run
`./scripts/dev doctor` against a running stack; it reads the server's Ogre log.
An accelerated GUI does not prove server acceleration.

- WSL2: enable Docker Desktop's Ubuntu integration. The runner automatically
  adds `compose.wslg.yaml` when WSLg sockets exist.
- Native Intel/AMD: select `COMPOSE_FILE=compose.yaml:compose.gpu.yaml` and set
  `RENDER_GID`/`VIDEO_GID` from your actual `/dev/dri` devices.
- NVIDIA: select `COMPOSE_FILE=compose.yaml:compose.nvidia.yaml` with host drivers
  and NVIDIA Container Toolkit. Add `compose.wslg.yaml` or `compose.x11.yaml`
  when a viewer needs that display path.
- Software fallback: `LIBGL_ALWAYS_SOFTWARE=1 ./scripts/dev mpc`. Expect slower
  startup and lower wall-time sensor rates.

Explicit `COMPOSE_FILE` overrides automatic selection; use the same files for
start, inspection, and stop. Native X11 needs `DISPLAY`, `/tmp/.X11-unix`, and
local Docker X access (for example `xhost +local:`). DRM group IDs must match the
Docker daemon host. For example, discover them using
`stat -c %g /dev/dri/renderD128` and `stat -c %g /dev/dri/card0` with real paths.

## Diagnose failures

```bash
./scripts/dev status
./scripts/dev mission status
./scripts/dev doctor
./scripts/dev logs
```

Inside `./scripts/dev shell`:

```bash
ros2 topic echo /uav1/mavros/state --once
ros2 topic echo /uav1/mpc/pose --qos-reliability best_effort --once
ros2 topic hz /uav1/scan
ros2 topic echo /mpc/setpoint_status --once
timeout 5s gz topic -e -t /world/mpc_small50/stats
```

Use the configured world name. Exit 124 from `timeout` is expected. A stopped
clock suggests paused/stalled physics or bridge/rendering failure. A slow clock
with progressing EKF/GPS messages means startup is advancing slowly. Connected
FCUs without poses need estimator and acknowledged telemetry investigation.
Missing upstream sources require submodule initialization; worker solve timeouts
require fixing the cause and restarting. For MAVROS allocator crashes, see the
[bootstrap constraint](DEVELOPMENT.md#mavros-bootstrap).
