#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

# If this script is launched from the Snap build of VS Code, GUI processes can
# inherit Snap GTK/GIO paths and make rviz2 load incompatible core20 libraries.
unset GTK_PATH
unset GTK_EXE_PREFIX
unset GIO_MODULE_DIR
unset GTK_IM_MODULE_FILE
unset LOCPATH
if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
fi

if ! ros2 pkg prefix g1_moveit_config >/dev/null 2>&1; then
  echo "g1_moveit_config is not available yet."
  echo "Run ./run/01_setup_assistant.sh first and save the generated package to src/g1_moveit_config."
  exit 1
fi

exec ros2 launch g1_moveit_config demo.launch.py
