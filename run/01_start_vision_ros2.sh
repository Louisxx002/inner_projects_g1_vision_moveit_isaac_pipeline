#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

require_file "${VISION_WS}/demo_realsense_realtime_lock_target.py" "vision detector"
require_file "${YOLO_MODEL}" "YOLO model"
require_file "${T_PELVIS_CAMERA}" "camera-to-pelvis calibration"
source_ros_base

args=(
  "${VISION_WS}/demo_realsense_realtime_lock_target.py"
  --model "${YOLO_MODEL}"
  --T-pelvis-camera "${T_PELVIS_CAMERA}"
  --locked-output "${TARGET_FILE}"
  --locked-hand-output "${TARGET_HAND_FILE}"
  --ros-topic "${ROS_TOPIC}"
  --ros-frame "${ROS_FRAME}"
)

if [[ -n "${TARGET_CLASS}" ]]; then
  args+=(--target-class "${TARGET_CLASS}")
fi
if [[ "${VISION_CONTINUOUS}" == "1" ]]; then
  args+=(--continuous)
fi
if [[ "${VISION_NO_DISPLAY}" == "1" ]]; then
  args+=(--no-display)
fi

cd "${VISION_WS}"
log "Starting RealSense + YOLO vision publisher on ${ROS_TOPIC}"
exec "${VISION_PYTHON}" "${args[@]}"
