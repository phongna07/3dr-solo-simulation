#!/usr/bin/env bash
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/solo_ws/install/setup.bash
# Only the development Compose service opts in to checkout imports.
if [[ "${SOLO_DEV:-0}" == 1 ]]; then
  export PYTHONPATH="/workspace/src/solo_mpc:/workspace/src/solo_bringup:${PYTHONPATH:-}"
fi
mkdir -p "${HOME}" /tmp/solo-runtime
chmod 700 /tmp/solo-runtime
export XDG_RUNTIME_DIR=/tmp/solo-runtime
if [[ -d /usr/share/gz ]]; then
  export GZ_VERSION=harmonic
  # ros_gz vendor hooks replace this path with bridge-only CLI definitions.
  # Retain the native Harmonic commands (including gz sim) alongside them.
  export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/solo_gazebo/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_GUI_PLUGIN_PATH="/opt/solo_gazebo/lib:${GZ_GUI_PLUGIN_PATH:-}"
  model_path=/opt/solo_ws/install/solo_mpc/share/solo_mpc/models
  if [[ "${SOLO_DEV:-0}" == 1 && -d /workspace/src/solo_mpc/models ]]; then
    model_path=/workspace/src/solo_mpc/models
  fi
  export SOLO_MODEL_PATH="${model_path}"
  export GZ_SIM_RESOURCE_PATH="${model_path}:${GZ_SIM_RESOURCE_PATH:-}"
  export SDF_PATH="${model_path}:${SDF_PATH:-}"
fi
exec "$@"
