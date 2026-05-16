#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
isaac_setup_common
isaac_prepare_cache
isaac_build_gui_args

ISAACSIM_EXEC="${G1_ISAACSIM_EXEC:-/workspace/g1_moveit_ws/isaacsim/open_g1_stage.py}"

docker run \
  "${ISAAC_RUN_ARGS[@]}" \
  "${ISAAC_ENV_ARGS[@]}" \
  "${ISAAC_MOUNT_ARGS[@]}" \
  "${ISAAC_IMAGE}" \
  -lc './isaac-sim.sh --/app/file/ignoreUnsavedOnExit=1 --/isaac/startup/ros_bridge_extension= --exec '"${ISAACSIM_EXEC}"' "$@"' \
  -- "$@"
