from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_file = LaunchConfiguration("target_file")
    arm = LaunchConfiguration("arm")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_file",
                default_value="/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt",
            ),
            DeclareLaunchArgument("arm", default_value="right"),
            Node(
                package="g1_grasp_planner",
                executable="moveit_plan_only_node",
                output="screen",
                arguments=[
                    "--target-file",
                    target_file,
                    "--arm",
                    arm,
                ],
            ),
        ]
    )
