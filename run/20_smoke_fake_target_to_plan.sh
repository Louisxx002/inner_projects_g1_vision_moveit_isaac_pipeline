#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

mkdir -p "${LOG_DIR}"
source_moveit_overlay
rm -f "${TRAJECTORY_JSON}"

moveit_pid=""
planner_pid=""

cleanup() {
  set +e
  if [[ -n "${planner_pid}" ]] && kill -0 "${planner_pid}" 2>/dev/null; then
    kill "${planner_pid}" 2>/dev/null
  fi
  if [[ -n "${moveit_pid}" ]] && kill -0 "${moveit_pid}" 2>/dev/null; then
    kill "${moveit_pid}" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

log "Starting MoveIt demo stack for smoke test"
"${SCRIPT_DIR}/02_start_moveit_demo.sh" >"${LOG_DIR}/smoke_moveit_demo.log" 2>&1 &
moveit_pid="$!"

sleep 8

log "Starting planner and waiting for fake ROS2 target"
"${SCRIPT_DIR}/03_plan_grasp_from_ros2.sh" >"${LOG_DIR}/smoke_plan.log" 2>&1 &
planner_pid="$!"

sleep 2

log "Publishing fake target on ${ROS_TOPIC}"
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  "${ROS_TOPIC}" geometry_msgs/msg/PointStamped \
  "{header: {frame_id: '${ROS_FRAME}'}, point: {x: 0.30, y: -0.20, z: 0.09}}" >/dev/null

wait "${planner_pid}"
planner_pid=""

require_file "${TRAJECTORY_JSON}" "planned trajectory"
log "Smoke plan succeeded: ${TRAJECTORY_JSON}"
