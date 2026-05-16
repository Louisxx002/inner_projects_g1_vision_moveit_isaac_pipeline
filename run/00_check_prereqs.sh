#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

log "Checking workspaces and files"
require_dir "${VISION_WS}" "vision workspace"
require_dir "${MOVEIT_WS}" "MoveIt workspace"
require_file "${VISION_WS}/demo_realsense_realtime_lock_target.py" "vision detector"
require_file "${YOLO_MODEL}" "YOLO model"
require_file "${T_PELVIS_CAMERA}" "camera-to-pelvis calibration"
require_file "${MOVEIT_WS}/run/20_isaacsim_gui_adaptive_grasp.sh" "Isaac playback script"

log "Checking Python environments"
[[ -x "${VISION_PYTHON}" ]] || die "vision Python is not executable: ${VISION_PYTHON}"
"${VISION_PYTHON}" - <<'PY'
import cv2
import numpy
import pyrealsense2
import ultralytics
print("vision python imports OK")
PY

log "Checking ROS and MoveIt overlay"
source_moveit_overlay
ros2 pkg prefix g1_grasp_planner >/dev/null
ros2 pkg prefix g1_moveit_config >/dev/null
ros2 pkg prefix moveit_ros_move_group >/dev/null

log "Checking target transfer files"
mkdir -p "$(dirname "${TARGET_FILE}")" "$(dirname "${TARGET_HAND_FILE}")" "${LOG_DIR}"
touch "${TARGET_FILE}" "${TARGET_HAND_FILE}"

log "OK"
