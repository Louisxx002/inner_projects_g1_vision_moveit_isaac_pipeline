#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

HAND_MODE="${G1_HAND_MODE:-inspire}"

exec ros2 run g1_grasp_planner hand_events_dry_run_bridge \
  --trajectory /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json \
  --hand-mode "${HAND_MODE}" \
  --report /home/louisxx/g1_moveit_ws/runtime/hand_events_dry_run_report.json
