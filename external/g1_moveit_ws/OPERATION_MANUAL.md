# G1 MoveIt 项目操作手册

本文档只记录“怎么运行并确认结果”。详细原理、代码结构和阶段说明见 `MOVEIT_WORKFLOW.md`、`FULL_SYSTEM_WORKFLOW.md`、`ISAAC_SIM_RUNBOOK.md`。

当前安全边界：

```text
当前只做目标点读取、MoveIt plan-only 规划、RViz/Isaac Sim 可视化、轨迹审查和 dry-run。
当前不向 Unitree 真机发送运动命令。
```

## 1. 常规自动选手流程

### 1.1 生成目标点

终端 1：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/01_run_vision_file.sh
```

目标点会写入：

```text
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
```

检查：

```bash
cat /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
cat /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
```

格式应为：

```text
x y z
```

坐标系应为 `pelvis`。

### 1.2 启动 MoveIt 和 RViz

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

保持这个终端打开。看到下面输出表示 MoveIt 已经可以规划：

```text
You can start planning now!
```

### 1.3 发布静态场景

终端 3：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
```

期望输出：

```text
Published static planning scene: table_keepout; removed torso_front_keepout
```

### 1.4 自动选择左手或右手规划抓取序列

默认模式是 `G1_ARM=auto`。MoveIt 会优先读取 `locked_target_hand.txt`，如果文件不存在，则按目标点 pelvis-frame `y` 值判断：`y > 0.02` 用左手，`y < -0.02` 用右手，中线附近需要手动指定。

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_grasp_sequence.sh
```

期望输出：

```text
Grasp sequence plan succeeded: ... stages, ... trajectory points, ... hand events
No execution was requested.
Saved grasp sequence trajectory: /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

这个序列包含：

```text
open hand event
pre_pick
pick
close hand hold
lift
pre_place
place
release hand hold
retreat
```

注意：当前手部开合是轨迹文件里的 `hand_events` 同步点，不会直接向 Inspire/Dex3 发 DDS 命令。

右手配置：

```text
group_name: right_arm
end_effector_link: right_hand_palm_link
pick_offset: [0.0, -0.02, 0.02]
```

左手配置：

```text
group_name: left_arm
end_effector_link: left_hand_palm_link
pick_offset: [0.0, 0.02, 0.02]
```

### 1.5 审查轨迹

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

必须看到：

```text
TRAJECTORY_REVIEW_PASSED
```

同时看 `joint_stats`，腰部不应大幅扭转。当前右手参考范围：

```text
waist_yaw_joint   max_abs 通常应小于 0.45 rad
waist_roll_joint  max_abs 通常应小于 0.25 rad
waist_pitch_joint max_abs 通常应小于 0.25 rad
```

如果只想调试单个抓取点可达性，而不是完整抓取序列，可以运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

### 1.6 在 RViz 看轨迹

如果 RViz 没自动显示或没看清：

```bash
cd /home/louisxx/g1_moveit_ws
./run/10_replay_last_trajectory_rviz.sh
```

RViz 中检查：

```text
Displays -> MotionPlanning -> Planned Path
```

确保 `MotionPlanning` 被勾选。

### 1.7 在 Isaac Sim 看轨迹

```bash
cd /home/louisxx/g1_moveit_ws
G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
```

Adaptive grasp demo:

```bash
./run/20_isaacsim_gui_adaptive_grasp.sh
```

成功时终端会出现：

```text
Isaac Sim Full App is loaded.
Playing trajectory: ... points, joints=[...]
Trajectory playback complete. Isaac Sim will stay open.
```

当前 Isaac Sim 只是轨迹播放验证，不是完整抓取仿真。还没有桌子、目标物体接触、闭合灵巧手、lift 等完整抓取流程。

## 2. 手动强制左右手

一般不需要手动指定。只有目标在中线附近、`locked_target_hand.txt` 输出 `center`，或者你明确要覆盖自动判断时，才使用下面的开关。

强制左手：

```bash
G1_ARM=left
```

强制右手：

```bash
G1_ARM=right
```

### 2.1 左手规划

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/05_plan_only_target.sh
```

左手配置：

```text
group_name: left_arm
end_effector_link: left_hand_palm_link
pick_offset: [0.0, 0.02, 0.02]
```

注意：左手需要目标点在左手合理工作区。右侧目标点直接强制切左手通常会失败。

### 2.2 左手审查、闸门和 dry-run

如果手动强制生成的是左手轨迹，后续也要带同一个开关。自动模式下不用额外加开关：

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
G1_ARM=left ./run/07_pre_execution_gate.sh
G1_ARM=left ./run/08_dry_run_unitree_bridge.sh
```

已知左手可规划测试点：

```text
locked target: [0.35, 0.20, 0.18]
left pick target: [0.35, 0.22, 0.20]
result: 23 trajectory points, TRAJECTORY_REVIEW_PASSED
```

## 3. 真机前 dry-run 检查

当前不接真机，只做 dry-run。

自动模式：

```bash
cd /home/louisxx/g1_moveit_ws
./run/07_pre_execution_gate.sh
./run/08_dry_run_unitree_bridge.sh
./run/09_verify_hardware_mapping.sh
```

手动强制左手：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/07_pre_execution_gate.sh
G1_ARM=left ./run/08_dry_run_unitree_bridge.sh
./run/09_verify_hardware_mapping.sh
```

必须看到：

```text
PRE_EXECUTION_GATE_PASSED
DRY_RUN_BRIDGE_READY
HARDWARE_MAPPING_PASSED
```

## 4. 一次完整实验命令

### 4.1 自动选手最常用命令

终端 1：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
./run/05_plan_grasp_sequence.sh
./run/06_review_last_trajectory.sh
./run/11_dry_run_hand_events.sh
./run/10_replay_last_trajectory_rviz.sh
G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
```

### 4.2 手动强制左手命令

终端 1：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
G1_ARM=left ./run/05_plan_grasp_sequence.sh
./run/06_review_last_trajectory.sh
G1_ARM=left ./run/11_dry_run_hand_events.sh
G1_ARM=left ./run/07_pre_execution_gate.sh
G1_ARM=left ./run/08_dry_run_unitree_bridge.sh
```

## 5. 常见问题

### 5.1 `MoveIt error_code=99999`

常见原因：

```text
move_group 没启动
目标点不可达
目标点在错误手臂一侧
目标太低或太靠近身体
同时存在多个 /move_action
MoveIt 配置未重新 build/restart
```

处理顺序：

```bash
pkill -f "ros2 launch g1_moveit_config demo.launch.py"
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
./run/03_demo_rviz.sh
```

另开终端：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
./run/05_plan_only_target.sh
```

### 5.2 轨迹成功但动作不自然

先看审查：

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

如果腰部大幅扭转，应看到：

```text
TRAJECTORY_REVIEW_FAILED
```

处理：

```text
不要直接放宽腰部约束。
优先调整目标点高度、侧向位置，或后续增加 pre-grasp 点。
```

### 5.3 Isaac 里手抖或左手乱动

当前播放脚本已经做了：

```text
逐帧插值轨迹
持续保持最终姿态
锁住所有非轨迹关节
```

如果仍抖动，原因通常在 Isaac USD 关节 drive/collision 设置，不是 MoveIt 轨迹本身。

### 5.4 左手规划失败

先确认日志是否已经切到左手：

```text
group=left_arm
link=left_hand_palm_link
```

如果已经切到左手但失败，优先检查目标点是否在左手工作区。右手目标点例如 `y=-0.12` 通常不适合左手。

### 5.5 目标点过期

重新运行视觉锁点，或测试时刷新文件时间：

```bash
touch /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

## 6. 当前成功标准

一次项目运行算成功，需要至少满足：

```text
MoveIt/RViz 成功启动
static scene 发布成功
05_plan_only_target 规划成功
06_review_last_trajectory 输出 TRAJECTORY_REVIEW_PASSED
RViz 能看到轨迹
Isaac Sim 能播放轨迹
dry-run 不向真机发命令
```

真机执行不在当前操作手册范围内。
