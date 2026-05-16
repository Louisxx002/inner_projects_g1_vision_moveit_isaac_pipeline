#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

exec ros2 run g1_grasp_planner verify_hardware_mapping \
  --trajectory /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json \
  --mapping /home/louisxx/g1_moveit_ws/config/unitree_g1_29_joint_map.yaml \
  --report /home/louisxx/g1_moveit_ws/runtime/hardware_mapping_report.json
