#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

ARM="${G1_ARM:-auto}"
TARGET_FILE="${G1_TARGET_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt}"
TARGET_HAND_FILE="${G1_TARGET_HAND_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt}"

exec ros2 run g1_grasp_planner moveit_grasp_sequence_node \
  --target-source file \
  --target-file "${TARGET_FILE}" \
  --target-hand-file "${TARGET_HAND_FILE}" \
  --arm "${ARM}" \
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
