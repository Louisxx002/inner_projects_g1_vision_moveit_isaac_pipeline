#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
isaac_setup_common
isaac_prepare_cache
isaac_build_headless_args

mkdir -p "${ISAAC_CACHE_HOST}/pkg"

docker run --name isaac-sim-g1 -it \
  "${ISAAC_RUN_ARGS[@]}" \
  "${ISAAC_ENV_ARGS[@]}" \
  "${ISAAC_MOUNT_ARGS[@]}" \
  "${ISAAC_IMAGE}"
