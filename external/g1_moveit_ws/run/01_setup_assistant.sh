#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_base

WORKSPACE="$(g1_moveit_ws_root)"
URDF="${WORKSPACE}/src/g1_moveit_config_seed/robot_description/g1_body29_hand14.urdf"
GROUP_NOTES="${WORKSPACE}/src/g1_moveit_config_seed/config/planning_groups.md"
cd "${WORKSPACE}/src/g1_moveit_config_seed/robot_description"

echo "Starting MoveIt Setup Assistant."
echo
echo "Use this URDF:"
echo "  $URDF"
echo
echo "Save generated config package to:"
echo "  $WORKSPACE/src/g1_moveit_config"
echo
echo "Planning group notes:"
echo "  $GROUP_NOTES"
echo
echo "Safety boundary: generate config only; do not connect robot execution."
echo

exec ros2 run moveit_setup_assistant moveit_setup_assistant
