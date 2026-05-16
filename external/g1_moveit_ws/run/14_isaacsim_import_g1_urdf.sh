#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
isaac_setup_common
isaac_prepare_cache

mkdir -p "${ISAAC_WORKSPACE_HOST}/runtime/isaac"
isaac_build_headless_args

docker run \
  "${ISAAC_RUN_ARGS[@]}" \
  "${ISAAC_ENV_ARGS[@]}" \
  "${ISAAC_MOUNT_ARGS[@]}" \
  "${ISAAC_IMAGE}" \
  -lc './python.sh /workspace/g1_moveit_ws/isaacsim/import_g1_urdf.py "$@"' \
  -- "$@"
