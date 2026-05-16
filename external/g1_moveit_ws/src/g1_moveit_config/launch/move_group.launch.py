from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(file_path="config/g1.urdf")
        .robot_description_semantic(file_path="config/g1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )
