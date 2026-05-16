#!/usr/bin/env bash
set -euo pipefail

source "${VISION_WS}/config/paths.env"

cd "${MOVEIT_WS}"

HAND_MODE="${G1_HAND_MODE:-${HAND_MODE}}"
LIVE_ARGS=()
if [[ "${ENABLE_LIVE_HAND:-0}" == "1" ]]; then
  LIVE_ARGS+=(--enable-live-hand)
fi

exec "${G1_PRIMITIVES_PYTHON}" -B \
  "${MOVEIT_WS}/src/g1_grasp_planner/g1_grasp_planner/hand_events_live_bridge.py" \
  --trajectory "${G1_TRAJECTORY_JSON:-${MOVEIT_WS}/runtime/last_plan_only_trajectory.json}" \
  --hand-mode "${HAND_MODE}" \
  "${LIVE_ARGS[@]}" \
  "$@"
