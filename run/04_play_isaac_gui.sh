#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

require_file "${TRAJECTORY_JSON}" "planned trajectory"

cd "${MOVEIT_WS}"
log "Launching Isaac Sim trajectory playback"
exec ./run/20_isaacsim_gui_adaptive_grasp.sh "$@"
