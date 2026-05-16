#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

exec ros2 run g1_grasp_planner verify_hardware_mapping \
  --trajectory "${G1_TRAJECTORY_JSON:-${MOVEIT_WS}/runtime/last_plan_only_trajectory.json}" \
  --mapping "${G1_MAPPING:-${MOVEIT_WS}/config/unitree_g1_29_joint_map.yaml}" \
  --report "${G1_HARDWARE_MAPPING_REPORT:-${MOVEIT_WS}/runtime/hardware_mapping_report.json}"
