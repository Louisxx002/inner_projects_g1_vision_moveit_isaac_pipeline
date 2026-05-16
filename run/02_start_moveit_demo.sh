#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

source_moveit_overlay

unset GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR GTK_IM_MODULE_FILE LOCPATH
if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
fi

cd "${MOVEIT_WS}"
log "Starting MoveIt demo stack. Keep this running while planning."
exec ros2 launch g1_moveit_config demo.launch.py
