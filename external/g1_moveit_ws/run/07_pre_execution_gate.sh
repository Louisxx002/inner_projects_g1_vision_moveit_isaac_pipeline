#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

ARM="${G1_ARM:-auto}"
TARGET_FILE="${G1_TARGET_FILE:-${VISION_WS}/runtime/locked_target_xyz.txt}"
TARGET_HAND_FILE="${G1_TARGET_HAND_FILE:-${VISION_WS}/runtime/locked_target_hand.txt}"

exec ros2 run g1_grasp_planner pre_execution_gate \
  --target-file "${TARGET_FILE}" \
  --target-hand-file "${TARGET_HAND_FILE}" \
  --trajectory "${G1_TRAJECTORY_JSON:-${MOVEIT_WS}/runtime/last_plan_only_trajectory.json}" \
  --review "${G1_REVIEW_JSON:-${MOVEIT_WS}/runtime/last_trajectory_review.json}" \
  --report "${G1_GATE_REPORT:-${MOVEIT_WS}/runtime/pre_execution_gate_report.json}" \
  --arm "${ARM}"
