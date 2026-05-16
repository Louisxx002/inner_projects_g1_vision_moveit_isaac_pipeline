from setuptools import setup

package_name = "g1_grasp_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/plan_only.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="louisxx",
    maintainer_email="louisxx@example.local",
    description="Safe plan-only MoveIt helpers for the G1 grasp pipeline.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "moveit_plan_only_node = g1_grasp_planner.moveit_plan_only_node:main",
            "moveit_grasp_sequence_node = g1_grasp_planner.moveit_grasp_sequence_node:main",
            "hand_events_dry_run_bridge = g1_grasp_planner.hand_events_dry_run_bridge:main",
            "pre_execution_gate = g1_grasp_planner.pre_execution_gate:main",
            "replay_trajectory_rviz = g1_grasp_planner.replay_trajectory_rviz:main",
            "review_trajectory = g1_grasp_planner.review_trajectory:main",
            "static_scene_publisher = g1_grasp_planner.static_scene_publisher:main",
            "trajectory_dry_run_bridge = g1_grasp_planner.trajectory_dry_run_bridge:main",
            "verify_hardware_mapping = g1_grasp_planner.verify_hardware_mapping:main",
        ],
    },
)
