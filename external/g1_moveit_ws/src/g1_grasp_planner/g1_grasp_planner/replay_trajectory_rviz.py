from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


def seconds_to_duration(value: float) -> Duration:
    duration = Duration()
    duration.sec = int(value)
    duration.nanosec = int(round((value - duration.sec) * 1e9))
    return duration


def load_display_trajectory(path: str | Path, model_id: str) -> DisplayTrajectory:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    summary = data["summary"]
    points = data["points"]

    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory.header.frame_id = "world"
    robot_trajectory.joint_trajectory.joint_names = list(summary["joint_names"])

    for point in points:
        msg_point = JointTrajectoryPoint()
        msg_point.positions = [float(value) for value in point["positions"]]
        msg_point.velocities = [float(value) for value in point["velocities"]]
        msg_point.accelerations = [float(value) for value in point["accelerations"]]
        msg_point.time_from_start = seconds_to_duration(float(point["time_from_start"]))
        robot_trajectory.joint_trajectory.points.append(msg_point)

    display = DisplayTrajectory()
    display.model_id = model_id
    display.trajectory_start = RobotState()
    display.trajectory.append(robot_trajectory)
    return display


class TrajectoryReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("g1_replay_trajectory_rviz")
        self.args = args
        self.display = load_display_trajectory(args.trajectory, args.model_id)
        self.publisher = self.create_publisher(DisplayTrajectory, args.topic, 10)

    def run(self) -> None:
        deadline = time.monotonic() + self.args.duration
        count = 0
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(self.display)
            count += 1
            self.get_logger().info(
                f"Published saved trajectory to {self.args.topic} "
                f"({len(self.display.trajectory[0].joint_trajectory.points)} points, count={count})"
            )
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(self.args.period)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay a saved plan-only trajectory to RViz display_planned_path.")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--topic", default="/display_planned_path")
    parser.add_argument("--model-id", default="g1")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--period", type=float, default=1.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = TrajectoryReplay(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
