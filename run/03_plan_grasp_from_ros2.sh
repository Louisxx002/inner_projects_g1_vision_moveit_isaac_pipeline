#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

source_moveit_overlay

mkdir -p "$(dirname "${TRAJECTORY_JSON}")"

cd "${MOVEIT_WS}"
log "Planning grasp sequence from ${ROS_TOPIC}"
exec ros2 run g1_grasp_planner moveit_grasp_sequence_node \
  --target-source ros2 \
  --ros-topic "${ROS_TOPIC}" \
  --wait-timeout "${G1_WAIT_TIMEOUT}" \
  --target-hand-file "${TARGET_HAND_FILE}" \
  --arm "${G1_ARM}" \
  --output-trajectory "${TRAJECTORY_JSON}" \
  --constrain-waist \
  --waist-path-constraints \
  --waist-yaw-tolerance 0.35 \
  --waist-roll-tolerance 0.20 \
  --waist-pitch-tolerance 0.20 \
  --constrain-wrist \
  --wrist-roll-tolerance 1.60 \
  --wrist-pitch-tolerance 1.45 \
  --wrist-yaw-tolerance 1.35 \
  --constrain-arm-posture \
  --shoulder-roll-tolerance 1.55 \
  --shoulder-yaw-tolerance 2.20 \
  --linear-substeps 2
