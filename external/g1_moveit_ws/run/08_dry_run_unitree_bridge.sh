#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

ARM="${G1_ARM:-auto}"
TARGET_FILE="${G1_TARGET_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt}"
TARGET_HAND_FILE="${G1_TARGET_HAND_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt}"

exec ros2 run g1_grasp_planner trajectory_dry_run_bridge \
  --target-file "${TARGET_FILE}" \
  --target-hand-file "${TARGET_HAND_FILE}" \
  --trajectory /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json \
  --review /home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json \
  --gate-report /home/louisxx/g1_moveit_ws/runtime/dry_run_bridge_gate_report.json \
  --mapping /home/louisxx/g1_moveit_ws/config/unitree_g1_29_joint_map.yaml \
  --arm "${ARM}"
