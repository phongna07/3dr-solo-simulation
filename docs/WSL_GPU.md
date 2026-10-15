# WSL, GUI, and GPU rendering

[Back to the project overview](../README.md)

Gazebo's server runs without a GUI by default, but the mission cameras and GPU
LiDAR still require rendering. A working display connection does not prove GPU
acceleration. Measure the renderer, sensor rates, and real-time factor before
running or comparing missions.

## Windows 11 with WSL2

Use Ubuntu WSL2, enable Docker Desktop's integration for that distribution, and
run `scripts/dev` from the Linux repository filesystem. Keep the repository under
`/home/<user>/...` rather than `/mnt/c/...`. VS Code's WSL extension can open the
project with `code .`.

When both `/mnt/wslg` and `/tmp/.X11-unix` exist, the CLI automatically adds
[compose.wslg.yaml](../compose.wslg.yaml). This maps display sockets and related
environment variables into the same container as the simulator. The entrypoint
creates a writable runtime directory. No `xhost +` is needed.

Attach a GUI to an existing stack:

```bash
./scripts/dev gui
```

Or use `./scripts/dev mpc --gui` to start the stack and attach after readiness.
Closing the client leaves the server running. WSLg supplies display access;
host GPU drivers and container GPU access require separate verification.

## Check rendering or use software fallback

From `./scripts/dev shell`:

```bash
glxinfo -B
gz sim --versions
timeout 5s gz topic -e -t /world/mpc_small50/stats
ros2 topic hz /uav1/camera/image_raw
ros2 topic hz /uav1/scan
```

`./scripts/dev doctor` checks container device access and reads the dedicated
`runs/latest/gazebo-home/.gz/rendering/ogre2.log` from the Gazebo server. A GUI
renderer does not substitute for this server check. Numeric GPU and serial group
IDs are required explicitly by the device overrides; use device ownership on the
Docker daemon host.

`glxinfo` helps inspect the display renderer; also check the Gazebo server's
render logs because its headless rendering path can differ. `llvmpipe` indicates
software rendering. Compare simulation and real time in Gazebo statistics;
simulation sensor rates can be much lower on wall time when real-time factor is
low.

After stopping the previous simulation, try a software fallback if hardware
rendering fails:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/dev mpc --gui
```

You can also set this variable in `.env`. Software rendering is functional on the
recorded host, but the three-camera/LiDAR mission ran at roughly 0.17–0.19 of real
time. See [Validation](VALIDATION.md) for the measured workload and its limits.

## Native Linux Intel or AMD

When `/dev/dri` exists on the Docker host, supply DRM devices and the actual group
IDs:

```bash
export COMPOSE_FILE=compose.yaml:compose.gpu.yaml
export RENDER_GID=$(stat -c %g /dev/dri/renderD128)
export VIDEO_GID=$(stat -c %g /dev/dri/card0)
./scripts/dev mpc
```

[compose.gpu.yaml](../compose.gpu.yaml) grants DRM access but does not configure
the display. A native Linux GUI needs a local override for X11 sockets, `DISPLAY`,
and Xauthority matching the host. Do not mount `/mnt/wslg` where it does not exist.

## NVIDIA

After confirming host driver and NVIDIA container GPU support, use the matching
override. For a WSLg host:

```bash
export COMPOSE_FILE=compose.yaml:compose.wslg.yaml:compose.nvidia.yaml
./scripts/dev mpc
```

On native Linux, omit the WSLg file and configure display access separately.
[compose.nvidia.yaml](../compose.nvidia.yaml) requests a GPU and graphics driver
capabilities; it does not install the host driver or toolkit. The recorded
validation does not confirm hardware acceleration for this setup. Do not assume
WSL devices are visible in Docker Desktop's VM.

## Override selection and networking

When `COMPOSE_FILE` is set, the CLI uses it as supplied and does not automatically
add WSLg. Include every override you need, and keep the same selection for start,
inspection, and stop. Use `unset COMPOSE_FILE` to return to automatic selection.

ROS and FCU links stay inside the simulation container, so host discovery and
Docker Desktop host networking are unnecessary for the default workflow. Access
ROS through `dev shell`. A Pi-to-laptop experiment needs explicit LAN endpoints
and discovery/routing changes; see [Hardware migration](HARDWARE.md#2-test-pi-to-laptop-sitl-before-connecting-solo).
