#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

mkdir -p "${LOG_DIR}"

vision_pid=""
moveit_pid=""

cleanup() {
  set +e
  if [[ -n "${vision_pid}" ]] && kill -0 "${vision_pid}" 2>/dev/null; then
    kill "${vision_pid}" 2>/dev/null
  fi
  if [[ -n "${moveit_pid}" ]] && kill -0 "${moveit_pid}" 2>/dev/null; then
    kill "${moveit_pid}" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

log "Starting vision publisher"
"${SCRIPT_DIR}/01_start_vision_ros2.sh" >"${LOG_DIR}/vision.log" 2>&1 &
vision_pid="$!"

log "Starting MoveIt demo stack"
"${SCRIPT_DIR}/02_start_moveit_demo.sh" >"${LOG_DIR}/moveit_demo.log" 2>&1 &
moveit_pid="$!"

log "Waiting for locked target on ${ROS_TOPIC}"
if ! wait_for_ros_topic_once "${ROS_TOPIC}" "${G1_WAIT_TIMEOUT}"; then
  die "No locked target received on ${ROS_TOPIC} within ${G1_WAIT_TIMEOUT}s. Check ${LOG_DIR}/vision.log"
fi

log "Target received; planning grasp sequence"
"${SCRIPT_DIR}/03_plan_grasp_from_ros2.sh" 2>&1 | tee "${LOG_DIR}/plan.log"

log "Planning completed; starting Isaac GUI playback"
"${SCRIPT_DIR}/04_play_isaac_gui.sh" "$@"
