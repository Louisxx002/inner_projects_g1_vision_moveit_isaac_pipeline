from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


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
    params = moveit_config.to_dict()
    rviz_config = f"{get_package_share_directory('g1_moveit_config')}/config/g1_moveit.rviz"

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
