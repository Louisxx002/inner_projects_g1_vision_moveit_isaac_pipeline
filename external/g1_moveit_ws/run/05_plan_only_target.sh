#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
ros2_setup_overlay

ARM="${G1_ARM:-auto}"
TARGET_FILE="${G1_TARGET_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt}"
TARGET_HAND_FILE="${G1_TARGET_HAND_FILE:-/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt}"
TARGET_SOURCE="${G1_TARGET_SOURCE:-file}"

case "${TARGET_SOURCE}" in
  file)
    exec ros2 run g1_grasp_planner moveit_plan_only_node \
      --target-source file \
      --target-file "${TARGET_FILE}" \
      --target-hand-file "${TARGET_HAND_FILE}" \
      --arm "${ARM}" \
      --constrain-waist \
      --waist-path-constraints \
      --waist-yaw-tolerance 0.35 \
      --waist-roll-tolerance 0.20 \
      --waist-pitch-tolerance 0.20
    ;;
  ros2)
    exec ros2 run g1_grasp_planner moveit_plan_only_node \
      --target-source ros2 \
      --ros-topic "${G1_TARGET_ROS_TOPIC:-/g1/locked_grasp_target}" \
      --wait-timeout "${G1_WAIT_TIMEOUT:-10}" \
      --target-hand-file "${TARGET_HAND_FILE}" \
      --arm "${ARM}" \
      --constrain-waist \
      --waist-path-constraints \
      --waist-yaw-tolerance 0.35 \
      --waist-roll-tolerance 0.20 \
      --waist-pitch-tolerance 0.20
    ;;
  *)
    echo "Unknown G1_TARGET_SOURCE: ${TARGET_SOURCE}" >&2
    exit 1
    ;;
esac
