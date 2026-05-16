from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rclpy
import yaml

from g1_grasp_planner.arm_config import (
    ARM_CHOICES,
    DEFAULT_HAND_DEADBAND_M,
    DEFAULT_TARGET_HAND_FILE,
)
from g1_grasp_planner.pre_execution_gate import StateValidityClient, run_file_checks
from g1_grasp_planner.verify_hardware_mapping import verify_mapping


def load_trajectory(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    summary = data.get("summary", {})
    points = data.get("points", [])
    if not summary or not points:
        raise ValueError(f"Invalid trajectory file: {path}")
    return data


def load_mapping(path: str | Path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid mapping file: {path}")
    return data


def run_gate(args: argparse.Namespace) -> tuple[bool, dict]:
    errors, report = run_file_checks(args)
    if not args.skip_moveit_state_check:
        rclpy.init()
        node = StateValidityClient()
        try:
            valid, state_errors = node.check_state_validity(args.group_name, args.state_timeout)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        report["moveit_state_valid"] = valid
        if state_errors:
            errors.extend(state_errors)
            report["errors"] = errors
    else:
        report["moveit_state_valid"] = None
        report["warnings"].append("MoveIt state validity check skipped")
    report["allowed"] = not errors
    return not errors, report


def format_vector(values: list[float], digits: int = 4) -> str:
    return "[" + ", ".join(f"{float(value):.{digits}f}" for value in values) + "]"


def print_send_plan(trajectory: dict, mapping: dict, mapped_rows: list[dict], max_preview_points: int) -> None:
    summary = trajectory["summary"]
    points = trajectory["points"]
    joints = summary["joint_names"]
    row_by_joint = {row["moveit_joint"]: row for row in mapped_rows}

    print("DRY_RUN_UNITREE_BRIDGE")
    print("No DDS initialization. No robot command will be sent.")
    print(f"control_mode: {mapping.get('control_mode')}")
    print(f"command_topic: {mapping.get('command_topic')}")
    print(f"state_topic: {mapping.get('state_topic')}")
    print(f"lower_body_controller: {mapping.get('lower_body_controller')}")
    print(f"weight_joint: {mapping.get('weight_joint')}")
    print(f"group_name: {summary['group_name']}")
    print(f"end_effector_link: {summary['end_effector_link']}")
    print(f"target_xyz: {summary['target_xyz']}")
    print(f"joint_count: {len(joints)}")
    print(f"point_count: {len(points)}")
    print(f"duration: {summary['duration']:.3f}s")
    print(f"max_abs_velocity: {summary['max_abs_velocity']:.6f} rad/s")
    print(f"max_abs_acceleration: {summary['max_abs_acceleration']:.6f} rad/s^2")
    print("joint_order:")
    for index, joint in enumerate(joints):
        row = row_by_joint[joint]
        print(f"  {index:02d}: {joint} -> motor_cmd[{row['unitree_index']}] {row['unitree_name']}")

    preview_count = min(max_preview_points, len(points))
    print(f"preview_points: {preview_count}/{len(points)}")
    for index, point in enumerate(points[:preview_count]):
        print(
            f"  point[{index:03d}] "
            f"t={float(point['time_from_start']):.3f}s "
            f"q={format_vector(point['positions'])}"
        )
        print(
            "    motor_cmd: "
            + ", ".join(
                f"{row_by_joint[joint]['unitree_index']}={float(value):.4f}"
                for joint, value in zip(joints, point["positions"])
            )
        )
    if len(points) > preview_count:
        last = points[-1]
        print(
            f"  point[{len(points) - 1:03d}] "
            f"t={float(last['time_from_start']):.3f}s "
            f"q={format_vector(last['positions'])}"
        )
        print(
            "    motor_cmd: "
            + ", ".join(
                f"{row_by_joint[joint]['unitree_index']}={float(value):.4f}"
                for joint, value in zip(joints, last["positions"])
            )
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dry-run Unitree trajectory bridge. Prints the send plan only.")
    parser.add_argument("--target-file", default="/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--review", default="/home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json")
    parser.add_argument("--gate-report", default="/home/louisxx/g1_moveit_ws/runtime/dry_run_bridge_gate_report.json")
    parser.add_argument("--mapping", default="/home/louisxx/g1_moveit_ws/config/unitree_g1_29_joint_map.yaml")
    parser.add_argument("--target-hand-file", default=DEFAULT_TARGET_HAND_FILE)
    parser.add_argument("--hand-deadband", type=float, default=DEFAULT_HAND_DEADBAND_M)
    parser.add_argument("--arm", choices=ARM_CHOICES, default="auto")
    parser.add_argument("--group-name", default=None)
    parser.add_argument("--pick-offset", nargs=3, type=float, default=None)
    parser.add_argument("--target-tolerance", type=float, default=1e-6)
    parser.add_argument("--max-target-age", type=float, default=3600.0)
    parser.add_argument("--max-trajectory-age", type=float, default=900.0)
    parser.add_argument("--max-review-age", type=float, default=900.0)
    parser.add_argument("--mtime-epsilon", type=float, default=0.01)
    parser.add_argument("--state-timeout", type=float, default=5.0)
    parser.add_argument("--skip-moveit-state-check", action="store_true")
    parser.add_argument("--max-preview-points", type=int, default=5)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    allowed, gate_report = run_gate(args)
    Path(args.gate_report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.gate_report).write_text(json.dumps(gate_report, indent=2), encoding="utf-8")

    print(f"gate_report: {Path(args.gate_report).resolve()}")
    if not allowed:
        print("DRY_RUN_BRIDGE_BLOCKED")
        for error in gate_report.get("errors", []):
            print(f"  - {error}")
        sys.exit(2)

    trajectory = load_trajectory(args.trajectory)
    mapping = load_mapping(args.mapping)
    mapping_errors, mapped_rows = verify_mapping(trajectory["summary"]["joint_names"], mapping)
    if mapping_errors:
        print("DRY_RUN_BRIDGE_BLOCKED")
        for error in mapping_errors:
            print(f"  - {error}")
        sys.exit(2)
    print_send_plan(trajectory, mapping, mapped_rows, args.max_preview_points)
    print("DRY_RUN_BRIDGE_READY")


if __name__ == "__main__":
    main()
