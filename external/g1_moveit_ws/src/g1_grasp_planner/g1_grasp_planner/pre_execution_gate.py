from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

from g1_grasp_planner.arm_config import (
    ARM_CHOICES,
    DEFAULT_HAND_DEADBAND_M,
    DEFAULT_TARGET_HAND_FILE,
    resolve_arm_config,
    resolve_requested_arm,
)
from g1_grasp_planner.safety import parse_xyz, read_xyz_file


class StateValidityClient(Node):
    def __init__(self) -> None:
        super().__init__("g1_pre_execution_gate")
        self.joint_state: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def _joint_state_cb(self, msg: JointState) -> None:
        self.joint_state = msg

    def wait_for_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self.joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.joint_state is not None

    def check_state_validity(self, group_name: str, timeout_sec: float) -> tuple[bool, list[str]]:
        errors: list[str] = []
        if not self.wait_for_joint_state(timeout_sec):
            return False, ["no /joint_states message received"]
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            return False, ["MoveIt /check_state_validity service is not available"]

        request = GetStateValidity.Request()
        request.robot_state.joint_state = self.joint_state
        request.group_name = group_name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, ["MoveIt /check_state_validity call timed out"]

        response = future.result()
        if not response.valid:
            for contact in response.contacts[:20]:
                errors.append(
                    "current state collision: "
                    f"{contact.contact_body_1} <-> {contact.contact_body_2} "
                    f"depth={contact.depth:.6f}"
                )
            if not errors:
                errors.append("current state is invalid")
            return False, errors
        return True, []


def file_age_sec(path: Path) -> float:
    return max(0.0, time.time() - path.stat().st_mtime)


def require_file(path: Path, errors: list[str]) -> None:
    if not path.exists():
        errors.append(f"missing file: {path}")
    elif path.stat().st_size <= 0:
        errors.append(f"empty file: {path}")


def close_enough(a: np.ndarray, b: np.ndarray, tolerance: float) -> bool:
    return bool(np.max(np.abs(a - b)) <= tolerance)


def run_file_checks(args: argparse.Namespace) -> tuple[list[str], dict]:
    errors: list[str] = []
    warnings: list[str] = []

    target_path = Path(args.target_file)
    trajectory_path = Path(args.trajectory)
    review_path = Path(args.review)
    for path in (target_path, trajectory_path, review_path):
        require_file(path, errors)
    if errors:
        return errors, {"warnings": warnings}

    target_age = file_age_sec(target_path)
    trajectory_age = file_age_sec(trajectory_path)
    review_age = file_age_sec(review_path)
    if target_age > args.max_target_age:
        errors.append(f"target file is stale: age={target_age:.1f}s max={args.max_target_age:.1f}s")
    if trajectory_age > args.max_trajectory_age:
        errors.append(f"trajectory file is stale: age={trajectory_age:.1f}s max={args.max_trajectory_age:.1f}s")
    if review_age > args.max_review_age:
        errors.append(f"review file is stale: age={review_age:.1f}s max={args.max_review_age:.1f}s")
    if review_path.stat().st_mtime + args.mtime_epsilon < trajectory_path.stat().st_mtime:
        errors.append("review report is older than trajectory file")

    review = json.loads(review_path.read_text(encoding="utf-8"))
    trajectory = json.loads(trajectory_path.read_text(encoding="utf-8"))
    if review.get("errors"):
        errors.append(f"trajectory review has errors: {review['errors']}")
    if str(Path(review.get("trajectory", "")).resolve()) != str(trajectory_path.resolve()):
        errors.append("review report does not reference the requested trajectory file")

    target = read_xyz_file(target_path)
    resolved_arm = resolve_requested_arm(args.arm, target, args.target_hand_file, args.hand_deadband)
    arm_config = resolve_arm_config(resolved_arm)
    args.arm = resolved_arm
    if args.group_name is None:
        args.group_name = arm_config.group_name
    if args.pick_offset is None:
        args.pick_offset = list(arm_config.pick_offset)

    summary = trajectory.get("summary", {})
    if summary.get("point_count", 0) < 2:
        errors.append("trajectory has fewer than 2 points")
    if summary.get("duration", 0.0) <= 0.0:
        errors.append("trajectory duration is not positive")
    if summary.get("joint_names") is None:
        errors.append("trajectory summary has no joint_names")
    if summary.get("group_name") != args.group_name:
        errors.append(
            f"trajectory group_name={summary.get('group_name')!r} does not match requested group_name={args.group_name!r}"
        )

    pick_offset = parse_xyz(args.pick_offset, "pick-offset")
    expected_target = target + pick_offset
    sequence_type = summary.get("sequence_type")
    target_key = "pick_xyz" if sequence_type == "grasp_plan_only" else "target_xyz"
    trajectory_target = np.array(summary.get(target_key, []), dtype=float)
    if trajectory_target.shape != (3,) or not np.all(np.isfinite(trajectory_target)):
        errors.append(f"trajectory {target_key} is invalid: {summary.get(target_key)}")
    elif not close_enough(expected_target, trajectory_target, args.target_tolerance):
        errors.append(
            "trajectory target does not match current target file: "
            f"expected={expected_target.tolist()} actual={trajectory_target.tolist()} "
            f"tolerance={args.target_tolerance}"
        )

    report = {
        "target_file": str(target_path.resolve()),
        "target_hand_file": str(Path(args.target_hand_file).resolve()),
        "arm": args.arm,
        "group_name": args.group_name,
        "trajectory": str(trajectory_path.resolve()),
        "review": str(review_path.resolve()),
        "target_age_sec": target_age,
        "trajectory_age_sec": trajectory_age,
        "review_age_sec": review_age,
        "expected_target_xyz": expected_target.tolist(),
        "trajectory_target_xyz": trajectory_target.tolist() if trajectory_target.shape == (3,) else summary.get("target_xyz"),
        "trajectory_point_count": summary.get("point_count"),
        "trajectory_duration": summary.get("duration"),
        "trajectory_sequence_type": sequence_type,
        "trajectory_target_key": target_key,
        "warnings": warnings,
        "errors": errors,
    }
    return errors, report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fail-closed pre-execution gate for a saved MoveIt trajectory.")
    parser.add_argument("--target-file", default="/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--review", default="/home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json")
    parser.add_argument("--report", default="/home/louisxx/g1_moveit_ws/runtime/pre_execution_gate_report.json")
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
    return parser


def main() -> None:
    args = build_parser().parse_args()
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

    allowed = not errors
    report["allowed"] = allowed
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"gate_report: {Path(args.report).resolve()}")
    print(f"target_age_sec: {report.get('target_age_sec')}")
    print(f"trajectory_age_sec: {report.get('trajectory_age_sec')}")
    print(f"review_age_sec: {report.get('review_age_sec')}")
    print(f"moveit_state_valid: {report.get('moveit_state_valid')}")

    if allowed:
        print("PRE_EXECUTION_GATE_PASSED")
        return

    print("PRE_EXECUTION_GATE_BLOCKED")
    for error in errors:
        print(f"  - {error}")
    sys.exit(2)


if __name__ == "__main__":
    main()
