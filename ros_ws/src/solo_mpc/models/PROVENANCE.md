# Vendored Iris assets and scenery

These are the minimal unchanged model resources used by the previous simulation
image, not a replacement vehicle mesh. They were imported on November 5, 2026
from `/opt/ardupilot_gazebo/models` in local image `solo-simulation:gd0`
(image ID
`sha256:cdbe415feae34d489be7560f6b3d8253223a88d1a3c1ce34dcc2105ce398b54f`).
Every unchanged imported file was checked
against its Git blob at the pinned upstream revision below. `SHA256SUMS` records
the exact vendored bytes, including the locally derived scenery.

- Repository: https://github.com/ArduPilot/ardupilot_gazebo
- Revision: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- Imported paths: `models/iris_with_standoffs/` and upstream `LICENSE.md`.
- `scenery.sdf` is derived from upstream `worlds/iris_runway.sdf`. It retains the
  original scene ambient/background/sky, sun lighting and attenuation, axes
  visuals, spherical coordinates, physics, and physics/user-commands/
  scene-broadcaster plugins. All includes, sensors, and sensors/IMU/NavSat
  plugins are removed. The world generator appends native UAVs and sensors.
- `iris_with_standoffs`: unchanged SDF, model metadata, body and propeller COLLADA
  meshes, and binary STL collision mesh. Original authors and PX4 model origin
  remain credited in `iris_with_standoffs/model.config`.
- License: the upstream repository supplies GNU LGPL version 3 in `LICENSE.md`,
  retained verbatim here. Author, origin, and contributor metadata inside the
  models and COLLADA files is retained unchanged, not relicensed by this project.

## Native-flight integration

The ArduPilot vehicle wrapper, ArduPilot plugin configuration, ArduPilot source,
upstream plugin binaries, and unused gimbal assets are deliberately not
vendored. The previous world generator removed the gimbal; only the unchanged
Iris geometry is needed to preserve the previous vehicle presentation. The world
generator flattens `iris_with_standoffs/model.sdf` and adds the native Gazebo Harmonic flight
systems; the original body/rotor geometry and material definitions stay intact.
No ArduPilot wrapper or external model download is needed to read the base model
or scenery.

`SOLO_MODEL_PATH`, `GZ_SIM_RESOURCE_PATH`, and `SDF_PATH` select the live
`/workspace/ros_ws/src/solo_mpc/models` directory in development and
`/opt/solo_ws/install/solo_mpc/share/solo_mpc/models` in the installed runtime.
The ROS package installs the entire tree, including license, provenance, and
checksums. Gazebo can resolve the unchanged `model://` mesh URIs without a
network download or the old `/opt/ardupilot_gazebo` directory.
