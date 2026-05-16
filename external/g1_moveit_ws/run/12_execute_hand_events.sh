#!/usr/bin/env bash
set -euo pipefail

source /home/louisxx/g1_grasp_pipeline_workspace/config/paths.env

cd /home/louisxx/g1_moveit_ws

HAND_MODE="${G1_HAND_MODE:-${HAND_MODE}}"
LIVE_ARGS=()
if [[ "${ENABLE_LIVE_HAND:-0}" == "1" ]]; then
  LIVE_ARGS+=(--enable-live-hand)
fi

exec "${G1_PRIMITIVES_PYTHON}" -B \
  /home/louisxx/g1_moveit_ws/src/g1_grasp_planner/g1_grasp_planner/hand_events_live_bridge.py \
  --trajectory /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json \
  --hand-mode "${HAND_MODE}" \
  "${LIVE_ARGS[@]}" \
  "$@"
