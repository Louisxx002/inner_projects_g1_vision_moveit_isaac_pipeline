from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, RobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from g1_grasp_planner.arm_config import (
    ARM_CHOICES,
    DEFAULT_HAND_DEADBAND_M,
    DEFAULT_TARGET_HAND_FILE,
    resolve_arm_config,
    resolve_requested_arm,
)
from g1_grasp_planner.moveit_plan_only_node import (
    DEFAULT_ROS_TOPIC,
    duration_to_sec,
    make_goal_constraints,
    make_stability_constraints,
    make_pose,
    wait_for_locked_target_file,
    wait_for_locked_target_ros2,
)
from g1_grasp_planner.safety import (
    DEFAULT_WORKSPACE_MAX,
    DEFAULT_WORKSPACE_MIN,
    parse_xyz,
    validate_workspace,
)


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
GRASP_WORKSPACE_ROOT = WORKSPACE_ROOT.parent / "inner_projects_g1_vision_grasp_pipeline"

DEFAULT_LEFT_PLACE = np.array([0.40, 0.18, 0.10], dtype=float)
DEFAULT_RIGHT_PLACE = np.array([0.40, -0.18, 0.10], dtype=float)


def zero_vector(size: int) -> list[float]:
    return [0.0] * size


def trajectory_final_positions(trajectory) -> list[float]:
    return list(trajectory.points[-1].positions)


def robot_state_from_positions(joint_names: list[str], positions: list[float]) -> RobotState:
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(joint_names)
    state.joint_state.position = list(positions)
    return state


class MoveItGraspSequenceClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("g1_moveit_grasp_sequence_client")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def plan_stage(self, stage_name: str, xyz: np.ndarray, start_state: RobotState | None):
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            raise RuntimeError("MoveIt /move_action server is not available")

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
        if start_state is not None:
            request.start_state = start_state
        if self.args.constrain_waist and self.args.waist_path_constraints:
            request.path_constraints = make_stability_constraints(self.args, name=f"{stage_name}_stability_path")

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        self.get_logger().info(f"Planning stage={stage_name} group={request.group_name} xyz={xyz.tolist()}")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"MoveIt rejected stage {stage_name}")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != 1:
            raise RuntimeError(f"Planning stage {stage_name} failed with MoveIt error_code={result.error_code.val}")

        trajectory = result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError(f"Planning stage {stage_name} returned no trajectory points")
        self.get_logger().info(f"Stage {stage_name} succeeded: {len(trajectory.points)} points")
        return trajectory


def append_stage_points(
    *,
    combined_points: list[dict],
    stages: list[dict],
    stage_name: str,
    target_xyz: np.ndarray,
    trajectory,
    time_offset: float,
    skip_first_point: bool,
) -> tuple[float, list[str], list[float]]:
    joints = list(trajectory.joint_names)
    stage_start_index = len(combined_points)
    source_points = trajectory.points[1:] if skip_first_point and len(trajectory.points) > 1 else trajectory.points

    for point in source_points:
        combined_points.append(
            {
                "time_from_start": time_offset + duration_to_sec(point.time_from_start),
                "positions": list(point.positions),
                "velocities": list(point.velocities) if point.velocities else zero_vector(len(joints)),
                "accelerations": list(point.accelerations) if point.accelerations else zero_vector(len(joints)),
                "stage": stage_name,
            }
        )

    stage_end_index = len(combined_points) - 1
    stage_end_time = combined_points[-1]["time_from_start"]
    stages.append(
        {
            "name": stage_name,
            "target_xyz": target_xyz.tolist(),
            "start_point_index": stage_start_index,
            "end_point_index": stage_end_index,
            "end_time": stage_end_time,
        }
    )
    return stage_end_time, joints, list(trajectory.points[-1].positions)


def append_hold_point(combined_points: list[dict], *, duration: float, stage: str) -> float:
    if not combined_points:
        raise ValueError("cannot append hold point before any trajectory point")
    previous = combined_points[-1]
    hold = {
        "time_from_start": float(previous["time_from_start"]) + float(duration),
        "positions": list(previous["positions"]),
        "velocities": zero_vector(len(previous["positions"])),
        "accelerations": zero_vector(len(previous["positions"])),
        "stage": stage,
    }
    combined_points.append(hold)
    return hold["time_from_start"]


def subdivide_stage(start: np.ndarray, end: np.ndarray, count: int) -> list[np.ndarray]:
    if count <= 1:
        return [end]
    return [start + (end - start) * (i / count) for i in range(1, count + 1)]


def write_grasp_sequence_json(
    path: str | Path,
    *,
    group_name: str,
    link_name: str,
    target_xyz: np.ndarray,
    pick_xyz: np.ndarray,
    place_xyz: np.ndarray,
    joint_names: list[str],
    points: list[dict],
    stages: list[dict],
    hand_events: list[dict],
) -> dict:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    max_abs_velocity = 0.0
    max_abs_acceleration = 0.0
    for point in points:
        max_abs_velocity = max(max_abs_velocity, max(abs(float(value)) for value in point["velocities"]))
        max_abs_acceleration = max(max_abs_acceleration, max(abs(float(value)) for value in point["accelerations"]))

    summary = {
        "sequence_type": "grasp_plan_only",
        "group_name": group_name,
        "end_effector_link": link_name,
        "target_xyz": target_xyz.tolist(),
        "pick_xyz": pick_xyz.tolist(),
        "place_xyz": place_xyz.tolist(),
        "joint_names": joint_names,
        "point_count": len(points),
        "stage_count": len(stages),
        "hand_event_count": len(hand_events),
        "duration": points[-1]["time_from_start"] if points else 0.0,
        "max_abs_velocity": max_abs_velocity,
        "max_abs_acceleration": max_abs_acceleration,
    }
    payload = {
        "summary": summary,
        "stages": stages,
        "hand_events": hand_events,
        "points": points,
    }
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plan a full MoveIt grasp sequence without robot execution.")
    parser.add_argument("--target-source", choices=("file", "ros2"), default="file")
    parser.add_argument("--target-file", default=str(GRASP_WORKSPACE_ROOT / "runtime" / "locked_target_xyz.txt"))
    parser.add_argument("--ros-topic", default=DEFAULT_ROS_TOPIC)
    parser.add_argument("--wait-timeout", type=float, default=None)
    parser.add_argument("--target-hand-file", default=DEFAULT_TARGET_HAND_FILE)
    parser.add_argument("--hand-deadband", type=float, default=DEFAULT_HAND_DEADBAND_M)
    parser.add_argument("--arm", choices=ARM_CHOICES, default="auto")
    parser.add_argument("--group-name", default=None)
    parser.add_argument("--end-effector-link", default=None)
    parser.add_argument("--frame-id", default="pelvis")
    parser.add_argument("--pick-offset", nargs=3, type=float, default=None)
    parser.add_argument("--place", nargs=3, type=float, default=None)
    parser.add_argument("--approach-z", type=float, default=0.10)
    parser.add_argument("--lift-z", type=float, default=0.12)
    parser.add_argument("--hand-close-duration", type=float, default=1.2)
    parser.add_argument("--hand-open-duration", type=float, default=0.8)
    parser.add_argument("--workspace-min", nargs=3, type=float, default=DEFAULT_WORKSPACE_MIN.tolist())
    parser.add_argument("--workspace-max", nargs=3, type=float, default=DEFAULT_WORKSPACE_MAX.tolist())
    parser.add_argument("--server-timeout", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.15)
    parser.add_argument("--acceleration-scale", type=float, default=0.15)
    parser.add_argument("--output-trajectory", default=str(WORKSPACE_ROOT / "runtime" / "last_plan_only_trajectory.json"))
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
    parser.add_argument("--linear-substeps", type=int, default=1)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    workspace_min = parse_xyz(args.workspace_min, "workspace-min")
    workspace_max = parse_xyz(args.workspace_max, "workspace-max")
    exit_code = 0

    rclpy.init()
    node = None
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
        place = parse_xyz(args.place, "place") if args.place is not None else (
            DEFAULT_LEFT_PLACE.copy() if resolved_arm == "left" else DEFAULT_RIGHT_PLACE.copy()
        )
        pre_pick = pick + np.array([0.0, 0.0, args.approach_z], dtype=float)
        lift = pick + np.array([0.0, 0.0, args.lift_z], dtype=float)
        pre_place = place + np.array([0.0, 0.0, args.approach_z], dtype=float)
        retreat = pre_place.copy()

        for name, xyz in (
            ("pick target", pick),
            ("pre-pick target", pre_pick),
            ("lift target", lift),
            ("place target", place),
            ("pre-place target", pre_place),
            ("retreat target", retreat),
        ):
            validate_workspace(name, xyz, workspace_min, workspace_max)

        node = MoveItGraspSequenceClient(args)
        combined_points: list[dict] = []
        stages: list[dict] = []
        hand_events: list[dict] = [{"event": "open", "hand": resolved_arm, "time_from_start": 0.0}]
        start_state = None
        time_offset = 0.0
        joint_names: list[str] | None = None
        last_positions: list[float] | None = None

        substeps = max(1, int(args.linear_substeps))
        stage_targets: list[tuple[str, np.ndarray]] = [("pre_pick", pre_pick)]
        stage_targets.extend(
            ((f"approach_{i}" if i < substeps else "pick", xyz)
             for i, xyz in enumerate(subdivide_stage(pre_pick, pick, substeps), start=1))
        )
        stage_targets.extend(
            ((f"lift_rise_{i}" if i < substeps else "lift", xyz)
             for i, xyz in enumerate(subdivide_stage(pick, lift, substeps), start=1))
        )
        stage_targets.append(("pre_place", pre_place))
        stage_targets.extend(
            ((f"lower_{i}" if i < substeps else "place", xyz)
             for i, xyz in enumerate(subdivide_stage(pre_place, place, substeps), start=1))
        )
        stage_targets.extend(
            ((f"retreat_rise_{i}" if i < substeps else "retreat", xyz)
             for i, xyz in enumerate(subdivide_stage(place, retreat, substeps), start=1))
        )

        for stage_name, xyz in stage_targets:
            trajectory = node.plan_stage(stage_name, xyz, start_state)
            stage_joints = list(trajectory.joint_names)
            if joint_names is None:
                joint_names = stage_joints
            elif stage_joints != joint_names:
                raise RuntimeError(f"stage {stage_name} joint_names changed: {stage_joints} != {joint_names}")

            time_offset, joint_names, last_positions = append_stage_points(
                combined_points=combined_points,
                stages=stages,
                stage_name=stage_name,
                target_xyz=xyz,
                trajectory=trajectory,
                time_offset=time_offset,
                skip_first_point=bool(combined_points),
            )

            if stage_name == "pick":
                hand_events.append({"event": "close", "hand": resolved_arm, "time_from_start": time_offset})
                time_offset = append_hold_point(combined_points, duration=args.hand_close_duration, stage="close_hand_hold")
            elif stage_name == "place":
                hand_events.append({"event": "release", "hand": resolved_arm, "time_from_start": time_offset})
                time_offset = append_hold_point(combined_points, duration=args.hand_open_duration, stage="release_hand_hold")

            start_state = robot_state_from_positions(joint_names, last_positions)

        if joint_names is None:
            raise RuntimeError("no trajectory was generated")

        summary = write_grasp_sequence_json(
            args.output_trajectory,
            group_name=args.group_name,
            link_name=args.end_effector_link,
            target_xyz=target_xyz,
            pick_xyz=pick,
            place_xyz=place,
            joint_names=joint_names,
            points=combined_points,
            stages=stages,
            hand_events=hand_events,
        )
        node.get_logger().info(
            "Grasp sequence plan succeeded: "
            f"{summary['stage_count']} stages, {summary['point_count']} points, "
            f"{summary['hand_event_count']} hand events, duration={summary['duration']:.3f}s. "
            "No robot execution was requested."
        )
        node.get_logger().info(f"Saved grasp sequence trajectory: {args.output_trajectory}")
    except Exception as exc:
        print(f"GRASP_SEQUENCE_PLAN_FAILED: {exc}", file=sys.stderr)
        exit_code = 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
