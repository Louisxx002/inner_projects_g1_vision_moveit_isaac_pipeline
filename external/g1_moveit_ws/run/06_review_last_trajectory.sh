#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

exec ros2 run g1_grasp_planner review_trajectory \
  --trajectory /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json \
  --joint-limits /home/louisxx/g1_moveit_ws/src/g1_moveit_config/config/joint_limits.yaml \
  --report /home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json
