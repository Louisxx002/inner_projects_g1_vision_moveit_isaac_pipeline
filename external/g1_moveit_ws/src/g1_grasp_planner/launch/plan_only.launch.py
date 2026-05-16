from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[5]
GRASP_WORKSPACE_ROOT = WORKSPACE_ROOT.parent / "inner_projects_g1_vision_grasp_pipeline"


def generate_launch_description():
    target_file = LaunchConfiguration("target_file")
    arm = LaunchConfiguration("arm")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_file",
                default_value=str(GRASP_WORKSPACE_ROOT / "runtime" / "locked_target_xyz.txt"),
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
