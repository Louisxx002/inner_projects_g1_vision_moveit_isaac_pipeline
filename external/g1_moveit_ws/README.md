# G1 MoveIt Planning Workspace

This workspace is for safe, plan-only path planning work. It is intentionally
separate from:

```text
/home/louisxx/g1_grasp_pipeline_workspace
```

Current safety boundary:

```text
No Unitree DDS commands.
No rt/arm_sdk publishing.
No robot execution bridge.
Plan and scene validation only.
```

Detailed Chinese workflow notes are in:

```text
MOVEIT_WORKFLOW.md
```

Step-by-step operation manual:

```text
OPERATION_MANUAL.md
```

Full system workflow, including grasp pipeline and Isaac Sim:

```text
FULL_SYSTEM_WORKFLOW.md
```

## 0. Install Required ROS Packages

This machine has ROS 2 Jazzy, but MoveIt 2 was not installed when this
workspace was created. Install it manually in a terminal:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-xacro \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

Verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_setup_assistant
```

## 1. Generate MoveIt Config

Use the Setup Assistant:

```bash
cd /home/louisxx/g1_moveit_ws
./run/01_setup_assistant.sh
```

Use this URDF:

```text
/home/louisxx/g1_moveit_ws/src/g1_moveit_config_seed/robot_description/g1_body29_hand14.urdf
```

Save the generated package as:

```text
/home/louisxx/g1_moveit_ws/src/g1_moveit_config
```

Use the joint group notes in:

```text
/home/louisxx/g1_moveit_ws/src/g1_moveit_config_seed/config/planning_groups.md
```

## 2. Build

After MoveIt is installed and `g1_moveit_config` is generated:

```bash
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
```

## 3. Validate in RViz Only

Start the generated MoveIt demo:

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

In RViz, use `Plan`, not `Execute`, until a separate execution bridge has been
reviewed and tested.

## 4. Static Scene Safety Test

After the MoveIt demo is running, publish conservative collision boxes:

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
```

This adds a table and simple forbidden volume to the MoveIt planning scene.

## 5. Plan-Only Target Test

With a locked target file from the existing vision pipeline:

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

By default this uses `G1_ARM=auto`: it reads
`/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt`
when available, otherwise infers left/right from the locked target `y` value.
Override with `G1_ARM=left` or `G1_ARM=right` when needed.

The node sends a MoveIt planning request with `plan_only=true`. It does not
execute the trajectory.

## Industrial Path To Execution

Only after RViz planning and collision checks are reliable:

```text
MoveIt JointTrajectory
  -> reviewed Unitree trajectory bridge
  -> rt/arm_sdk
```

Do not connect execution until these fail-closed rules exist:

```text
planning failure -> no motion
collision result -> no motion
stale target -> no motion
joint limit violation -> no motion
velocity/acceleration violation -> no motion
DDS/control timeout -> stop
operator stop -> stop
```
