from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from g1_grasp_planner.arm_config import (
    ARM_CHOICES,
    DEFAULT_HAND_DEADBAND_M,
    DEFAULT_TARGET_HAND_FILE,
    resolve_arm_config,
    resolve_requested_arm,
)
from g1_grasp_planner.safety import (
    DEFAULT_WORKSPACE_MAX,
    DEFAULT_WORKSPACE_MIN,
    parse_xyz,
    read_xyz_file,
    validate_workspace,
)


DEFAULT_ROS_TOPIC = "/g1/locked_grasp_target"


@dataclass
class LockedTarget:
    xyz: np.ndarray
    frame_id: str


def make_pose(frame_id: str, xyz: np.ndarray) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(xyz[0])
    pose.pose.position.y = float(xyz[1])
    pose.pose.position.z = float(xyz[2])
    pose.pose.orientation.w = 1.0
    return pose


def make_waist_constraints(args: argparse.Namespace, *, name: str) -> Constraints:
    constraints = Constraints()
    constraints.name = name
    for joint_name, tolerance in (
        ("waist_yaw_joint", args.waist_yaw_tolerance),
        ("waist_roll_joint", args.waist_roll_tolerance),
        ("waist_pitch_joint", args.waist_pitch_tolerance),
    ):
        joint = JointConstraint()
        joint.joint_name = joint_name
        joint.position = 0.0
        joint.tolerance_above = float(tolerance)
        joint.tolerance_below = float(tolerance)
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def make_stability_constraints(args: argparse.Namespace, *, name: str) -> Constraints:
    constraints = make_waist_constraints(args, name=name)
    if not getattr(args, "constrain_wrist", False):
        return constraints

    if "left" in args.group_name:
        prefix = "left"
    elif "right" in args.group_name:
        prefix = "right"
    else:
        return constraints

    if getattr(args, "constrain_arm_posture", False):
        for joint_name, tolerance in (
            (f"{prefix}_shoulder_roll_joint", args.shoulder_roll_tolerance),
            (f"{prefix}_shoulder_yaw_joint", args.shoulder_yaw_tolerance),
        ):
            joint = JointConstraint()
            joint.joint_name = joint_name
            joint.position = 0.0
            joint.tolerance_above = float(tolerance)
            joint.tolerance_below = float(tolerance)
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

    for joint_name, tolerance in (
        (f"{prefix}_wrist_roll_joint", args.wrist_roll_tolerance),
        (f"{prefix}_wrist_pitch_joint", args.wrist_pitch_tolerance),
        (f"{prefix}_wrist_yaw_joint", args.wrist_yaw_tolerance),
    ):
        joint = JointConstraint()
        joint.joint_name = joint_name
        joint.position = 0.0
        joint.tolerance_above = float(tolerance)
        joint.tolerance_below = float(tolerance)
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def make_goal_constraints(
    group_name: str,
    link_name: str,
    target_pose: PoseStamped,
    args: argparse.Namespace,
) -> Constraints:
    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [0.025]

    pos = PositionConstraint()
    pos.header = target_pose.header
    pos.link_name = link_name
    pos.constraint_region.primitives.append(region)
    pos.constraint_region.primitive_poses.append(target_pose.pose)
    pos.weight = 1.0

    ori = OrientationConstraint()
    ori.header = target_pose.header
    ori.link_name = link_name
    ori.orientation = target_pose.pose.orientation
    ori.absolute_x_axis_tolerance = 3.14
    ori.absolute_y_axis_tolerance = 3.14
    ori.absolute_z_axis_tolerance = 3.14
    ori.weight = 0.1

    constraints = Constraints()
    constraints.name = f"{group_name}_target"
    constraints.position_constraints.append(pos)
    constraints.orientation_constraints.append(ori)
    if args.constrain_waist:
        constraints.joint_constraints.extend(make_stability_constraints(args, name="stability_goal").joint_constraints)
    return constraints


def duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def write_trajectory_json(path: str | Path, *, group_name: str, link_name: str, xyz: np.ndarray, trajectory) -> dict:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    joints = list(trajectory.joint_names)
    points = []
    max_abs_velocity = 0.0
    max_abs_acceleration = 0.0
    for point in trajectory.points:
        velocities = list(point.velocities)
        accelerations = list(point.accelerations)
        if velocities:
            max_abs_velocity = max(max_abs_velocity, max(abs(value) for value in velocities))
        if accelerations:
            max_abs_acceleration = max(max_abs_acceleration, max(abs(value) for value in accelerations))
        points.append(
            {
                "time_from_start": duration_to_sec(point.time_from_start),
                "positions": list(point.positions),
                "velocities": velocities,
                "accelerations": accelerations,
            }
        )

    summary = {
        "group_name": group_name,
        "end_effector_link": link_name,
        "target_xyz": xyz.tolist(),
        "joint_names": joints,
        "point_count": len(points),
        "duration": points[-1]["time_from_start"] if points else 0.0,
        "max_abs_velocity": max_abs_velocity,
        "max_abs_acceleration": max_abs_acceleration,
    }
    payload = {
        "summary": summary,
        "points": points,
    }
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return summary


def wait_for_locked_target_file(path: str, frame_id: str, timeout_sec: float | None = None) -> LockedTarget:
    target_path = Path(path)
    start_time = time.monotonic()
    last_error: Exception | None = None

    while True:
        if target_path.exists():
            try:
                xyz = read_xyz_file(target_path)
                return LockedTarget(xyz=xyz, frame_id=frame_id)
            except Exception as exc:
                last_error = exc

        if timeout_sec is not None and time.monotonic() - start_time > timeout_sec:
            detail = f" Last read error: {last_error}" if last_error is not None else ""
            raise TimeoutError(f"Timed out waiting for locked target file {target_path}.{detail}")
        time.sleep(0.1)


def wait_for_locked_target_ros2(topic: str, timeout_sec: float | None = None) -> LockedTarget:
    from geometry_msgs.msg import PointStamped
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    node = rclpy.create_node("g1_moveit_locked_target_subscriber")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received: dict[str, LockedTarget] = {}

    def callback(msg: PointStamped) -> None:
        received["target"] = LockedTarget(
            xyz=np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float),
            frame_id=msg.header.frame_id or "pelvis",
        )

    node.create_subscription(PointStamped, topic, callback, qos)
    node.get_logger().info(f"Waiting for locked target on ROS2 topic: {topic}")
    start_time = time.monotonic()
    try:
        while "target" not in received:
            rclpy.spin_once(node, timeout_sec=0.1)
            if timeout_sec is not None and time.monotonic() - start_time > timeout_sec:
                raise TimeoutError(f"Timed out waiting for locked target on {topic}")
        target = received["target"]
        node.get_logger().info(f"Received locked target frame={target.frame_id!r} xyz={target.xyz.tolist()}")
        return target
    finally:
        node.destroy_node()


class MoveItPlanOnlyClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("g1_moveit_plan_only_client")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def plan(self, xyz: np.ndarray) -> int:
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            self.get_logger().error("MoveIt /move_action server is not available")
            return 2

        target_pose = make_pose(self.args.frame_id, xyz)
        request = MotionPlanRequest()
        request.group_name = self.args.group_name
        request.num_planning_attempts = self.args.planning_attempts
        request.allowed_planning_time = self.args.allowed_planning_time
        request.max_velocity_scaling_factor = self.args.velocity_scale
        request.max_acceleration_scaling_factor = self.args.acceleration_scale
        request.goal_constraints.append(
            make_goal_constraints(self.args.group_name, self.args.end_effector_link, target_pose, self.args)
        )
        if self.args.constrain_waist and self.args.waist_path_constraints:
            request.path_constraints = make_stability_constraints(self.args, name="stability_path")

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        self.get_logger().info(
            "Sending plan-only request "
            f"group={request.group_name} link={self.args.end_effector_link} xyz={xyz.tolist()}"
        )
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveIt rejected the planning goal")
            return 3

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != 1:
            self.get_logger().error(f"Planning failed with MoveIt error_code={error_code}")
            return 4

        points = result.planned_trajectory.joint_trajectory.points
        joints = result.planned_trajectory.joint_trajectory.joint_names
        summary = None
        if self.args.output_trajectory:
            summary = write_trajectory_json(
                self.args.output_trajectory,
                group_name=self.args.group_name,
                link_name=self.args.end_effector_link,
                xyz=xyz,
                trajectory=result.planned_trajectory.joint_trajectory,
            )
        self.get_logger().info(
            f"Plan succeeded: {len(points)} trajectory points, joints={list(joints)}. "
            "No execution was requested."
        )
        if summary is not None:
            self.get_logger().info(
                "Saved plan-only trajectory: "
                f"{self.args.output_trajectory} "
                f"duration={summary['duration']:.3f}s "
                f"max_abs_velocity={summary['max_abs_velocity']:.3f}rad/s "
                f"max_abs_acceleration={summary['max_abs_acceleration']:.3f}rad/s^2"
            )
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plan-only MoveIt request from a locked target file or ROS2 topic.")
    parser.add_argument("--target-source", choices=("file", "ros2"), default="file")
    parser.add_argument("--target-file", default="/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt")
    parser.add_argument("--ros-topic", default=DEFAULT_ROS_TOPIC)
    parser.add_argument("--wait-timeout", type=float, default=None)
    parser.add_argument("--target-hand-file", default=DEFAULT_TARGET_HAND_FILE)
    parser.add_argument("--hand-deadband", type=float, default=DEFAULT_HAND_DEADBAND_M)
    parser.add_argument("--arm", choices=ARM_CHOICES, default="auto")
    parser.add_argument("--group-name", default=None)
    parser.add_argument("--end-effector-link", default=None)
    parser.add_argument("--frame-id", default="pelvis")
    parser.add_argument("--pick-offset", nargs=3, type=float, default=None)
    parser.add_argument("--workspace-min", nargs=3, type=float, default=DEFAULT_WORKSPACE_MIN.tolist())
    parser.add_argument("--workspace-max", nargs=3, type=float, default=DEFAULT_WORKSPACE_MAX.tolist())
    parser.add_argument("--server-timeout", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.15)
    parser.add_argument("--acceleration-scale", type=float, default=0.15)
    parser.add_argument("--output-trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--constrain-waist", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--waist-path-constraints", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--waist-yaw-tolerance", type=float, default=0.35)
    parser.add_argument("--waist-roll-tolerance", type=float, default=0.20)
    parser.add_argument("--waist-pitch-tolerance", type=float, default=0.20)
    parser.add_argument("--constrain-wrist", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--wrist-roll-tolerance", type=float, default=1.20)
    parser.add_argument("--wrist-pitch-tolerance", type=float, default=1.05)
    parser.add_argument("--wrist-yaw-tolerance", type=float, default=1.20)
    parser.add_argument("--constrain-arm-posture", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--shoulder-roll-tolerance", type=float, default=1.55)
    parser.add_argument("--shoulder-yaw-tolerance", type=float, default=1.65)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    workspace_min = parse_xyz(args.workspace_min, "workspace-min")
    workspace_max = parse_xyz(args.workspace_max, "workspace-max")

    rclpy.init()
    try:
        if args.target_source == "ros2":
            target = wait_for_locked_target_ros2(args.ros_topic, args.wait_timeout)
        else:
            target = wait_for_locked_target_file(args.target_file, args.frame_id, args.wait_timeout)

        target_xyz = target.xyz
        target_frame_id = target.frame_id or args.frame_id
        if target_frame_id != args.frame_id:
            raise ValueError(
                f"Locked target frame_id={target_frame_id!r} does not match expected frame_id={args.frame_id!r}. "
                "Transform the target before planning."
            )
        validate_workspace("locked target", target_xyz, workspace_min, workspace_max)

        resolved_arm = resolve_requested_arm(args.arm, target_xyz, args.target_hand_file, args.hand_deadband)
        arm_config = resolve_arm_config(resolved_arm)
        args.arm = resolved_arm
        if args.group_name is None:
            args.group_name = arm_config.group_name
        if args.end_effector_link is None:
            args.end_effector_link = arm_config.end_effector_link
        if args.pick_offset is None:
            args.pick_offset = list(arm_config.pick_offset)

        pick_offset = parse_xyz(args.pick_offset, "pick-offset")
        pick = target_xyz + pick_offset
        validate_workspace("pick target", pick, workspace_min, workspace_max)

        node = MoveItPlanOnlyClient(args)
        try:
            exit_code = node.plan(pick)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
