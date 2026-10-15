# Simulation operations

[Back to the project overview](../README.md)

Complete the [first-run setup](../README.md#first-run) before using this guide.
Run host commands from the repository root. Commands under “Inspect ROS and
Gazebo” and “Record telemetry” run inside `./scripts/dev shell`.

## Mission control

| Command | Behavior |
|---|---|
| `./scripts/dev mpc` | Initialize the stack, take off, supervise tracking, LAND, and await disarm |
| `./scripts/dev mpc --gui` | Run the mission with a GUI alongside it |
| `./scripts/dev mpc --no-wait` | Skip host readiness progress; the mission still waits for initialization |
| `./scripts/dev mpc --seconds 20` | Limit tracking to 20 wall seconds after takeoff and controller initialization |
| `./scripts/dev mpc --record-cameras` | Also record one FOV camera video per UAV in the run directory |
| `./scripts/dev mpc --record-overhead` | Also record a 1920 × 1080 elevated target-following video |
| `./scripts/dev wait 600` | Wait up to 600 wall seconds for the current stack |
| `./scripts/dev mission run` | Take off, supervise tracking, LAND, and wait for disarm |
| `./scripts/dev mission run --seconds 20` | Limit tracking to 20 wall seconds after initialization |
| `./scripts/dev mission start` | Take off and enable tracking, then return to the caller |
| `./scripts/dev mission status` | Print controller diagnostics |
| `./scripts/dev mission stop` | Release publishing, pause the target, LAND, and wait for disarm |
| `./scripts/dev mission reset` | Reset the target to the start of its route and pause it |
| `./scripts/dev stop` | Run mission stop successfully before stopping the container |
| `./scripts/dev stop --force` | Stop simulation processes immediately, without LAND |

`mpc` automatically runs the mission after all launched processes, MPC nodes,
camera detectors, and requested recorders initialize. It checks fresh fleet
readiness and common-frame alignment before takeoff. `--no-wait` skips the host
progress display, while retaining the mission's startup gate. A failed or exited
startup process prevents arming. No second terminal command is needed to fly.
The lower-level `mission run` command remains available for another flight in
an existing stack after the previous mission has landed.
It keeps the target paused and the setpoint publisher disabled during takeoff,
then waits for usable controller inputs before enabling tracking. It finishes
when the target and fleet reach the route endpoint, or the requested duration
expires. Controller/publisher failure, prolonged target loss, loss of
GUIDED/armed/connection state, or departure from the altitude envelope triggers
an attempt to land.

During camera occlusion the target estimate briefly coasts, then the controller
searches near the last plausible position. The supervisor pauses the simulated
target during prolonged loss and aborts if it cannot be reacquired in time. This
policy assumes a cooperative simulation target.

`mission start` returns after enabling tracking. It does not leave the mission
supervisor running in the background, and does not record the supervisor's
`truth.jsonl`. The caller must monitor the mission and invoke `mission stop`.
Prefer `mpc` for supervised experiments and independent evaluation.

Ctrl+C, terminal closure, or termination of the `mpc` wrapper requests cancellation
through the mounted run directory. Its Docker client stays attached while the
supervisor attempts cleanup and landing, including interruption during takeoff.
Forced termination such as SIGKILL cannot run this handler. Ctrl+C directly in
the lower-level `mission run` command still depends on interrupt delivery through
Docker. Check for errors and confirm
disarm; cleanup can fail if required services or processes have already stopped.
Normal `dev stop` leaves the container running if mission stop fails. Use
`stop --force` for a crashed or failed simulation when landing cannot complete.
Hardware shutdown is different; see [Hardware migration](HARDWARE.md).

After landing, normal container shutdown sends SIGINT to ROS launch so its
children can finish cleanup and close the AVI writers before exiting. A direct
SIGTERM of the ROS launch process can leave buffered video files incomplete.

Closing the GUI only closes its client. Ctrl+C in `dev logs` only stops log
following. Stop an existing run before launching another; every `dev mpc` creates
new simulator state, and MPC runs do not support resume.

Both `dev mpc --gui` and `dev gui` automatically follow the pink target in MPC
worlds. The viewpoint sits 24 m above it, offset 10 m along each horizontal axis,
and looks down with a wide view of the formation and nearby obstacles. Press
Escape in the 3D view to release camera follow and navigate freely. Reopen the
GUI to restore the target view.

## Topology and readiness

One `sim` container contains Gazebo, the fleet's SITL/MAVROS instances, readiness,
sensor bridges, and MPC nodes. ROS discovery is localhost within the container;
FCU ports are not published to the host. Inspect ROS through `dev shell`.

| UAV | SITL instance | SYSID | JSON UDP | MAVLink TCP | MAVROS namespace |
|---|---:|---:|---:|---:|---|
| uav1 | 0 | 1 | 9002 | 5760 | /uav1/mavros |
| uav2 | 1 | 2 | 9012 | 5770 | /uav2/mavros |
| uav3 | 2 | 3 | 9022 | 5780 | /uav3/mavros |

Readiness uses typed ROS subscriptions and services. For each UAV it requires:

1. A connected FCU and successfully applied MAVROS plugin parameters.
2. Acknowledged requests for `LOCAL_POSITION_NED` (ID 32) and `ATTITUDE` (ID 30)
   at 10 Hz. Requests are serialized per FCU.
3. At least three finite pose messages with distinct timestamps, with the latest
   received less than 10 wall seconds ago. FCU state must be less than 20 wall
   seconds old for hardware, or 60 wall seconds for simulation heartbeats.

The monitor writes live `health.json`, updates `ready.json` while ready, and
publishes `/solo/readiness`. Docker health uses the live report. On disconnect,
telemetry/pose counters reset and are requested again after reconnect. Initial
readiness has a 600 s default deadline; timeout stops the stack. After becoming
ready, the monitor keeps reporting losses and recovery without arming or
restarting an FCU.

The host readiness helper polls container state every two seconds, rejects health
from an earlier container start, and reports exited stacks promptly. Readiness
confirms telemetry flow; it does not prove prearm acceptance, localization quality,
sensor performance, or flight safety. Mission checks add alignment and controller
input requirements.

Gazebo, SITL, MAVROS, readiness, and the sensor bridge are critical processes;
exit of one shuts down the launch. MPC/perception/target adapter exit leaves the
flight stack alive so supervised cleanup can attempt LAND. No automatic FCU
respawn occurs during an experiment.

## Inspect ROS and Gazebo

On the host:

```bash
./scripts/dev status
./scripts/dev mission status
./scripts/dev logs
./scripts/dev shell
```

Inside the container:

```bash
ros2 topic echo /uav1/mavros/state --once
ros2 topic echo /uav1/mavros/local_position/pose --qos-reliability best_effort --once
ros2 topic hz /uav1/mavros/local_position/pose
ros2 topic hz /uav1/scan
ros2 topic hz /uav1/camera/image_raw
ros2 topic echo /solo/readiness --once
ros2 topic echo /mpc/setpoint_status --once
timeout 5s gz topic -e -t /world/mpc_small50/stats
gz sdf -k /workspace/runs/latest/world.sdf
```

Exit code 124 from the timed Gazebo command is expected. Compare simulation time
with wall time to measure real-time factor. Requested sensor/telemetry rates are
not necessarily achieved on wall time. Multiple terminals can each use `dev shell`;
there is no need to launch extra MAVROS instances for observation.

## Logs and evaluation

| Run output | Contents |
|---|---|
| `manifest.json`, `source-sha256.json` | Mode, fleet, source versions, commit, and file hashes |
| `fleet.yaml`, `scenario.yaml`, `controller.yaml`, `mpc-config.json` | Snapshot and resolved experiment configuration |
| `world.sdf`, `bridge.yaml`, `uavN-mavros.yaml` | Generated world, sensor bridge, and MAVROS configuration |
| `alignment.json` | Frozen world-to-local calibration |
| `health.json`, `ready.json` | Current health and most recent ready report |
| `logs/`, `sitl/<instance>/` | ROS/process logs and each SITL's EEPROM/flight logs |
| `controller_sources.jsonl` | Source hashes and resolved configuration at controller startup |
| `mpc.jsonl` | Measured state, target estimates, solver outcomes, and timings |
| `truth.jsonl` | Independent Gazebo poses recorded by `mission run` |
| `evaluation.json` | Derived geometry, tracking, compute, and simulation-speed metrics |
| `cameras/<uav-name>.avi`, `cameras/<uav-name>.timestamps.jsonl` | Optional UAV camera videos and original per-frame simulation timestamps |
| `overhead/overhead.avi`, `overhead/overhead.timestamps.jsonl` | Optional 1080p elevated scene video and original per-frame simulation timestamps |

`runs/latest` selects the most recent simulation; launching another run changes
that link. The actual image ID is stored beside the run directory in
`runs/<run-id>.image-id.json`. Preserve the specific run directory for comparisons.
`runs/` and `artifacts/` are Git-ignored and must be shared separately if needed.

Evaluate while the simulation container is still running, after the mission lands:

```bash
./scripts/dev evaluate
./scripts/dev evaluate --run-dir /workspace/runs/<run-id>
```

Evaluation prefers independent Gazebo geometry when `truth.jsonl` has suitable
samples; otherwise it uses MAVROS measurements and labels `geometry_source`.
The current airborne filter requires every UAV above 2.5 m, so revisit evaluation
when changing the default 3 m altitude. Sampled metrics do not establish continuous
collision avoidance; inspect physics contacts and camera tracking too.

## Record UAV camera video

Enable recording when starting a fresh simulation:

```bash
./scripts/dev mpc --record-cameras --gui --seconds 20
./scripts/dev stop
ls runs/latest/cameras/
```

`--gui` is optional. Recording uses each UAV's `/uavN/camera/image_raw` topic,
so the files show the drone's downward FOV rather than the Gazebo GUI view.
All vehicles in the selected fleet are recorded in a separate ROS process;
recording is disabled unless `--record-cameras` is supplied. Live source mounting
allows this option to work with the existing simulation image without a rebuild.
For direct ROS launch, use `record_cameras:=true`.

The host files are `runs/<run-id>/cameras/<uav-name>.avi` (MJPEG). The recorder
subscribes during startup but discards frames until that UAV is connected, armed,
and more than 0.15 m above its disarmed local ground altitude, using fresh MAVROS
state and pose telemetry. Arming while sitting on the ground does not start a
video. Recordings include the rest of takeoff, tracking, and landing, and close
when the simulation stops. Closing the GUI or completing the mission
leaves recording active. Use `dev stop` to finish and finalize the videos.

Resolution and playback FPS come from the camera settings (default 160 × 160,
8 FPS). Each received frame is written once; playback follows the configured
simulation camera rate, independent of wall-clock simulation speed. Missed frames
shorten playback; the adjacent `.timestamps.jsonl` records each zero-based video
frame index and its original ROS simulation timestamp in nanoseconds (`stamp_ns`).
No video is created for a camera that never publishes. Abrupt process termination
or a host crash can leave incomplete videos. Recorder errors appear in `dev logs`
and do not shut down the flight stack.

## Record overhead video

Start a fresh simulation with the elevated scene recording enabled:

```bash
./scripts/dev mpc --record-overhead --seconds 20
./scripts/dev stop
ls runs/latest/overhead/
```

The video follows the pink target from 24 m above, offset by -10 m on both
horizontal axes and looking toward it, similar to the default GUI viewpoint.
It renders directly at **1920 × 1080 (16:9), 20 FPS**; the GUI's window size
does not affect the recording. It works without opening the GUI. Add `--gui`
to watch, or combine `--record-overhead --record-cameras` to save both the scene
and the UAV FOVs. Navigating or closing the GUI does not change the scene video.

Open `runs/<run-id>/overhead/overhead.avi` in your desktop file explorer after
`dev stop` finalizes it. It uses MJPEG and starts when the first connected, armed
UAV rises more than 0.15 m above its disarmed ground altitude. Startup frames
are discarded. It continues through takeoff, tracking, and landing until
the stack stops. Playback uses simulation time; dropped frames shorten the
video. `overhead.timestamps.jsonl` preserves the received simulation timestamps.
The extra full-HD camera and encoding consume rendering, CPU, and disk capacity;
recording is disabled unless requested. No image rebuild is needed. For direct
MPC ROS launch, set `record_overhead:=true`. Recorder errors appear in `dev logs`
without stopping the flight stack.

## Record telemetry

Inside the container, use a new output directory for each recording:

```bash
ros2 bag record -s mcap -o /workspace/runs/latest/telemetry \
  /uav1/mavros/state /uav1/mavros/local_position/pose \
  /uav2/mavros/state /uav2/mavros/local_position/pose \
  /uav3/mavros/state /uav3/mavros/local_position/pose /solo/readiness
```

Adjust topics for your fleet, and stop the recorder with Ctrl+C before stopping
the container. If pose messages are absent, check sensor QoS and use a recording
QoS override as needed. Measure storage and CPU costs before adding raw video.

## Troubleshooting

| Symptom | Check or action |
|---|---|
| Docker socket permission denied | Confirm Docker is running and WSL integration is enabled |
| Stack exits while `mpc` waits | Read the reported process logs; a stopped container cannot become ready |
| TCP opens but MAVROS stays disconnected | Inspect SITL defaults, PANIC messages, and JSON sensor input; an open port does not prove FCU boot |
| Connected without pose | Check telemetry ACKs, EKF, and real-time factor in the readiness report |
| `PreArm: Gyros inconsistent` | Check model overlap, IMU scope, stable physics, and simulator load before retrying |
| `setpoint_attitude` / `invalid allocator` crash | Use the current parameter bootstrap; see the pinned MAVROS callback limitation in [Development](DEVELOPMENT.md#mavros-parameter-bootstrap) |
| Wrong MAVROS runtime version | Verify the entrypoint sources `/opt/mavros_ws/install/setup.bash`, not just the ROS base |
| EGL/OpenGL errors or slow sensors | Inspect the renderer using [WSL and GPU](WSL_GPU.md); increasing timeouts does not fix rates |
| Container becomes unhealthy after readiness | Inspect live `health.json` for stale pose/state or a disconnected FCU |
| `No complete airborne fleet records` during evaluation | Check whether takeoff succeeded and whether altitude matches the evaluator's 2.5 m filter |
| LAND cleanup fails | Inspect FCU state and service errors; normal stop preserves the container for recovery |
