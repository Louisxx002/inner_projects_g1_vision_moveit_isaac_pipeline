# inner_projects_g1_vision_moveit_isaac_pipeline

This workspace is the integration layer for:

```text
RealSense + YOLO
  -> /g1/locked_grasp_target
  -> MoveIt grasp sequence planning
  -> Isaac Sim trajectory playback
```

It integrates the vision workspace with a MoveIt/Isaac planning stack:

- `inner_projects_g1_vision_grasp_pipeline`: vision, YOLO, RealSense, target locking
- `external/g1_moveit_ws/`: MoveIt planning and Isaac playback source. Build it in place, or point `MOVEIT_WS` at an existing overlay.

## Check

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
./run/00_check_prereqs.sh
```

## Run Step By Step

Terminal 1:

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
./run/01_start_vision_ros2.sh
```

Terminal 2:

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
./run/02_start_moveit_demo.sh
```

Terminal 3:

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
./run/03_plan_grasp_from_ros2.sh
```

Terminal 4:

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
xhost +local:docker
./run/04_play_isaac_gui.sh
```

## One Command

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
xhost +local:docker
./run/10_full_vision_moveit_isaac.sh
```

The one-command script starts vision and MoveIt in the background, waits for a
locked ROS2 target, plans the grasp sequence, then opens Isaac Sim playback.
Logs are written to `logs/`.

## Smoke Test Without Camera

This starts MoveIt, publishes one fake ROS2 target, and verifies that the
MoveIt planner can produce `last_plan_only_trajectory.json`.

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_vision_moveit_isaac_pipeline
./run/20_smoke_fake_target_to_plan.sh
```

## Useful Overrides

```bash
TARGET_CLASS=bottle ./run/01_start_vision_ros2.sh
G1_WAIT_TIMEOUT=60 ./run/10_full_vision_moveit_isaac.sh
VISION_NO_DISPLAY=1 ./run/01_start_vision_ros2.sh
```
