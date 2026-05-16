#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
isaac_setup_common
isaac_prepare_cache
isaac_build_gui_args

docker run \
  "${ISAAC_RUN_ARGS[@]}" \
  "${ISAAC_ENV_ARGS[@]}" \
  "${ISAAC_MOUNT_ARGS[@]}" \
  "${ISAAC_IMAGE}" \
  -lc './python.sh /workspace/g1_moveit_ws/isaacsim/play_last_trajectory.py \
      --robot-usd /workspace/g1_moveit_ws/runtime/isaac/g1.usd \
      --gui-kinematic \
      --keep-open \
      "$@"' \
  -- "$@"
