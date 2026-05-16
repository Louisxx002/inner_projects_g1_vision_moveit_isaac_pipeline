#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

exec ros2 run g1_grasp_planner review_trajectory \
  --trajectory "${G1_TRAJECTORY_JSON:-${MOVEIT_WS}/runtime/last_plan_only_trajectory.json}" \
  --joint-limits "${G1_JOINT_LIMITS:-${MOVEIT_WS}/src/g1_moveit_config/config/joint_limits.yaml}" \
  --report "${G1_REVIEW_JSON:-${MOVEIT_WS}/runtime/last_trajectory_review.json}"
