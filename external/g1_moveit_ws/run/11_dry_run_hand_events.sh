#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

HAND_MODE="${G1_HAND_MODE:-inspire}"

exec ros2 run g1_grasp_planner hand_events_dry_run_bridge \
  --trajectory "${G1_TRAJECTORY_JSON:-${MOVEIT_WS}/runtime/last_plan_only_trajectory.json}" \
  --hand-mode "${HAND_MODE}" \
  --report "${G1_HAND_EVENTS_DRY_RUN_REPORT:-${MOVEIT_WS}/runtime/hand_events_dry_run_report.json}"
