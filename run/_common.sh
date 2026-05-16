#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FULL_PIPELINE_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${FULL_PIPELINE_WS}/config/paths.env"

log() {
  printf '[g1-full] %s\n' "$*"
}

die() {
  printf '[g1-full] ERROR: %s\n' "$*" >&2
  exit 1
}

source_ros_base() {
  [[ -f "${ROS_SETUP}" ]] || die "ROS setup not found: ${ROS_SETUP}"
  set +u
  source "${ROS_SETUP}"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
}

source_moveit_overlay() {
  source_ros_base
  [[ -f "${MOVEIT_WS}/install/setup.bash" ]] || die "MoveIt overlay not built: ${MOVEIT_WS}/install/setup.bash"
  set +u
  source "${MOVEIT_WS}/install/setup.bash"
  set -u
}

moveit_workspace_root() {
  if [[ -n "${MOVEIT_WS:-}" ]]; then
    printf '%s\n' "${MOVEIT_WS}"
    return
  fi
  printf '%s\n' "${FULL_PIPELINE_WS}/external/g1_moveit_ws"
}

require_file() {
  [[ -f "$1" ]] || die "$2 missing: $1"
}

require_dir() {
  [[ -d "$1" ]] || die "$2 missing: $1"
}

maybe_target_class_args() {
  if [[ -n "${TARGET_CLASS}" ]]; then
    printf '%s\0%s\0' --target-class "${TARGET_CLASS}"
  fi
}

wait_for_ros_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  source_ros_base
  timeout "${timeout_sec}" ros2 topic echo --once "${topic}" >/dev/null
}
