# Hardware migration: Raspberry Pi 4, 3DR Solo, LD19, and GoPro

[Back to the project overview](../README.md)

This guide preserves the path from simulation to physical UAVs. **The hardware
profile has not been validated on a real Solo or Pi.** It starts MAVROS and
readiness only, with no automatic firmware flashing, arming, or sensor startup.
The mission CLI rejects hardware runs, and `companion-stop` stops software
without sending LAND.

## What is available and what is missing

| Area | Current support | Work required for hardware |
|---|---|---|
| Companion image | ROS Jazzy, pinned MAVROS, readiness, rosbag, camera libraries, project packages | ARM64 build and runtime validation on Pi |
| FCU link | Configurable MAVROS `fcu_url` | Verified Solo firmware, wiring/routing, baud, SYSID, command ownership |
| MPC | Source in `solo_mpc`; numerical dependencies in the simulation image | Install ARM64 numerical dependencies, add hardware launch/adapters, profile solver |
| Localization | MAVROS local pose; simulation alignment uses ground truth | Real estimator, common origin/yaw, calibration, time synchronization |
| LD19 | Disabled configuration placeholder | Pin, build, and integrate a ROS 2 driver |
| GoPro | Disabled configuration placeholder | Select a supported model/protocol, decode stream, publish calibrated images |
| Mission / failsafe | Simulation supervision | Hardware flight policy, FCU failsafes, pilot override, staged flight validation |

The companion image contains project packages but lacks the simulation image's
MPC numerical dependencies, Gazebo, SITL, and ArduPilot build environment. The
presence of `solo_mpc` source does not make it a hardware flight stack. Enabling
`ld19` or `gopro` in the current hardware configuration causes launch to fail
explicitly until drivers are integrated.

## 1. Prepare the Pi environment

Use a 64-bit host OS compatible with ROS Jazzy and the selected sensor drivers;
Ubuntu Server 24.04 ARM64 is the baseline to evaluate. Host responsibilities
include kernel/UART configuration, udev names, networking, clock synchronization,
power, and thermal management. Confirm camera SDK requirements before finalizing
the host OS.

After preparing `.env`, build natively on the Pi:

```bash
./scripts/dev build companion
```

Or build an ARM64 image on a Buildx-capable machine:

```bash
docker buildx build --platform linux/arm64 --target companion \
  -f docker/Dockerfile -t solo-companion:gd0-arm64 --load .
```

Transfer the image to the Pi using a registry or `docker save` / `docker load`,
and set `COMPANION_IMAGE` to the chosen tag or digest. An emulated build does not
validate runtime performance. Keep ARM64 manifests separately from amd64;
see [Reproducibility](REPRODUCIBILITY.md).

## 2. Test Pi-to-laptop SITL before connecting Solo

Use the laptop as the simulator and the Pi as the companion to validate CPU
budget, networking, and ROS/FCU interfaces first. This topology still requires
implementation; the default simulation keeps FCU links inside one container.

1. Choose a TCP or UDP MAVLink endpoint, LAN addresses, ports, and SYSIDs.
2. For TCP, make SITL listen on a reachable container interface and publish its
   port through Compose. The Pi connects to the laptop's reachable LAN address,
   not `127.0.0.1`. Verify firewall and WSL/Docker routing.
3. Replace the corresponding laptop MAVROS control connection, or use a MAVLink
   router with explicit ownership rules. Avoid competing control publishers or
   trying to share a single TCP connection without routing.
4. If ROS sensor/state traffic crosses the LAN, configure DDS discovery beyond
   `LOCALHOST`, then verify connectivity, QoS, timestamps, and packet loss.
5. Add real-time hardware clock handling and a measured-state/frame adapter;
   the current simulation adapters depend on Gazebo time and alignment data.

This stage can test the Pi controller against SITL once dependencies and launch
integration are complete. It does not validate physical Solo dynamics or sensors.

## 3. Verify the Solo and connect MAVROS

Record the actual FCU board, bootloader, firmware, Solo/OpenSolo version, and
existing parameter set before deciding whether to upgrade. The simulation's
ArduCopter 4.5.3 pin is not a firmware compatibility guarantee for a particular
Solo. Consult the [ArduPilot Solo developer documentation](https://ardupilot.org/dev/docs/solo.html)
and [OpenSolo releases](https://github.com/OpenSolo/OpenSolo/releases) for the
installed hardware/software combination.

ArduPilot runs on the FCU; the Pi runs companion software. Preserve the Solo's
existing i.MX6/controller link. Choose a verified UART or routed MAVLink link,
including electrical compatibility and port ownership. Check the carrier's actual
pinout rather than assuming a generic TELEM2 mapping.

Edit [config/hardware.yaml](../config/hardware.yaml) to match the FCU's SYSID and
link. The template `/dev/solo-fcu:921600` is a placeholder, not a measured Solo
baud rate. Set a stable host device path using a verified udev rule. For a serial
link on native Linux Pi:

```bash
export COMPOSE_FILE=compose.yaml:compose.hardware-serial.yaml
export FCU_DEVICE=/dev/solo-fcu
export DIALOUT_GID=$(getent group dialout | cut -d: -f3)
./scripts/dev companion
docker compose --profile hardware logs -f companion
```

`FCU_DEVICE` is the host path; the override maps it to `/dev/solo-fcu` inside the
container. Replace the host path if your stable device name differs. The profile
uses host networking and a narrow device mapping with the dialout group. It does
not require a privileged container. USB access under WSL requires a separate
passthrough setup and is not the Pi baseline.

Hardware logs and health reports are under `runs/hardware/<run-id>/`, with
`runs/hardware/latest` selecting the latest companion run. Readiness requires
local pose as well as a heartbeat. A connected FCU without GPS or another fused
localization source may never become ready; camera images and LiDAR scans do not
automatically supply EKF pose.

To stop the companion software:

```bash
./scripts/dev companion-stop
```

This command does not land the drone. Implement and validate FCU/companion
link-loss, stale-state, solver-stall, and shutdown policies before physical flight.

## 4. Integrate LD19

Start with the manufacturer's [ldlidar_stl_ros2 driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2).
Pin a commit after confirming it builds and runs on Jazzy ARM64. Test sensor
output independently over USB–serial before integrating flight control.

Add the verified device to Compose, extend hardware launch, and publish
`/uavN/scan` with the expected `LaserScan` semantics and a calibrated
`uavN/laser` frame. Use a stable udev name based on the adapter's actual identity.
Measure scan rate, packet loss, range limits, timestamps, and extrinsics relative
to `base_link`. Match simulation range/rate/noise to the real sensor and mounting.

LD19 provides a 2D scan plane that tilts with a body-mounted sensor. It is not a
complete 3D obstacle map or a full localization source. Preserve invalid/no-return
ray distinctions and tilt rejection when replacing the Gazebo scan adapter.

## 5. Integrate GoPro recording and image input

The intended workflow records video to the GoPro SD card while supplying images
to Pi perception:

```text
GoPro → SD recording
      → stream/capture → decode → ROS Image + CameraInfo → perception
```

Identify the HERO model and firmware before choosing a protocol. Check
[Open GoPro](https://gopro.github.io/OpenGoPro/) and its
[streaming FAQ](https://gopro.github.io/OpenGoPro/docs/faq/) for supported modes.
Concurrent recording and streaming remains unverified for this project. Do not
assume a USB connection provides UVC `/dev/video0`; verify network-over-USB,
Wi-Fi, or HDMI capture support for the selected model.

Begin with a low-resolution perception stream and, where supported, separate
high-quality SD recording. Measure end-to-end latency, buffering, frame drops,
and CPU load while MPC and LiDAR are active. Publish `/uavN/camera/image_raw` and
`/uavN/camera/camera_info` with measured intrinsics and a calibrated optical frame.
Replace the simulation's magenta-marker/flat-plane assumptions as required by
the physical target.

Receipt time is not exposure time. Establish synchronization and measure delay
before using images for localization or control. Gimbal motion and stabilization
can change extrinsics; do not use a fixed camera transform for a moving mount.

## 6. Validate before expanding flight scope

1. Verify ARM64 boot/restart, stable power, cooling, and no automatic arming.
2. With propellers removed, test SYSID/modes, reconnect, command ownership,
   pilot override, and FCU failsafes alongside the existing Solo link.
3. Validate localization, common ENU origin/yaw, sensor timestamps, intrinsics,
   extrinsics, and ENU/NED setpoint conversion on the actual firmware.
4. Measure solver p95/p99, state age, publish jitter, deadline misses, and
   link-loss behavior under full load on the Pi.
5. Identify vehicle response to position/velocity/acceleration references and
   validate saturation and altitude control.
6. Progress through simple position flight, single-UAV MPC, then multiple UAVs
   with common localization and validated separation behavior.

Record firmware, parameters, source/configuration snapshots, image digest, and
measured outcomes at each stage. The [simulation results](VALIDATION.md) support
software integration only; they do not certify these hardware stages.
