# G1 MoveIt 工作流程文档

本文档说明 `/home/louisxx/g1_moveit_ws` 这个 MoveIt 工作区从 0 到当前状态做了什么、解决了什么问题、现在能实现什么效果，以及每次运行应该怎么操作。

当前边界很重要：

```text
当前只做 MoveIt 规划、轨迹保存、轨迹审查、RViz 可视化和真机执行前 dry-run。
当前不向 Unitree 真机发送 DDS / rt/arm_sdk 控制命令。
```

## 1. 项目目标

这个工作区的目标是把原来 `g1_grasp_pipeline_workspace` 识别出来的目标点，交给 MoveIt 做机械臂路径规划。

整体关系是：

```text
g1_grasp_pipeline_workspace
  RealSense / YOLO / 深度点 / 手眼标定
  -> locked_target_xyz.txt + locked_target_hand.txt 或 ROS2 PointStamped

g1_moveit_ws
  读取目标点并自动选择左/右臂
  -> MoveIt plan-only 规划
  -> RViz 看轨迹
  -> 保存 JointTrajectory JSON
  -> 审查速度、加速度、碰撞和目标新鲜度
  -> Unitree motor index 映射 dry-run
```

所以这个项目不是替代识别端，而是补上“目标点到可执行关节轨迹”这一段。

## 2. 当前已经做了什么

### 2.1 建立独立 MoveIt 工作区

创建了独立工作区：

```text
/home/louisxx/g1_moveit_ws
```

这样做的原因是：

- 不污染原来的视觉抓取项目。
- ROS2 Jazzy / MoveIt 依赖和视觉端 conda 环境分开。
- 真机执行前可以先在 MoveIt/RViz/Isaac Sim 里验证。

### 2.2 准备 G1 MoveIt 配置

当前 MoveIt 配置包是：

```text
/home/louisxx/g1_moveit_ws/src/g1_moveit_config
```

核心文件：

```text
config/g1.urdf
config/g1.srdf
config/kinematics.yaml
config/joint_limits.yaml
config/ompl_planning.yaml
config/g1_moveit.rviz
launch/demo.launch.py
launch/move_group.launch.py
```

当前主要规划组：

```text
right_arm
left_arm
dual_arm
```

当前已跑通的是：

```text
right_arm + right_hand_palm_link
left_arm + left_hand_palm_link
```

### 2.3 修复 MoveIt 启动问题

之前遇到过：

```text
package 'joint_state_publisher' not found
```

处理方式是安装缺失 ROS 包。

之后又遇到：

```text
Planning plugin name is empty or not defined in namespace 'ompl'
```

处理方式是在：

```text
src/g1_moveit_config/config/ompl_planning.yaml
```

补上：

```yaml
planning_plugins:
  - ompl_interface/OMPLPlanner
```

并配置了 request/response adapters、RRTConnect planner。

### 2.4 修复 RViz 显示轨迹

配置了：

```text
src/g1_moveit_config/config/g1_moveit.rviz
```

并让：

```text
src/g1_moveit_config/launch/demo.launch.py
```

启动 RViz 时加载该配置。RViz 现在会订阅：

```text
/display_planned_path
```

用于显示规划轨迹。

### 2.5 增加静态场景

新增节点：

```text
src/g1_grasp_planner/g1_grasp_planner/static_scene_publisher.py
```

运行脚本：

```bash
./run/04_static_scene.sh
```

当前效果：

```text
发布 table_keepout
移除 torso_front_keepout
```

原因是之前 `torso_front_keepout` 太粗，会误挡手腕，导致规划失败。

### 2.6 增加目标点规划节点

新增节点：

```text
src/g1_grasp_planner/g1_grasp_planner/moveit_plan_only_node.py
```

它做的事情：

- 从文件或 ROS2 topic 获取锁定目标点。
- 检查目标点是否在工作空间范围内。
- 给目标点加抓取偏移。
- 向 MoveIt `/move_action` 发送 plan-only 请求。
- 不执行轨迹。
- 把规划结果保存为 JSON。

默认参数：

```text
target-source: file
target-file: /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
target-hand-file: /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
ros-topic: /g1/locked_grasp_target
arm: auto
right arm group/link: right_arm / right_hand_palm_link
left arm group/link: left_arm / left_hand_palm_link
frame-id: pelvis
right pick-offset: [0.0, -0.02, 0.02]
left pick-offset: [0.0, 0.02, 0.02]
hand-deadband: 0.02
velocity-scale: 0.15
acceleration-scale: 0.15
allowed-planning-time: 5.0
planning-attempts: 5
constrain-waist: enabled by run/05_plan_only_target.sh
waist-yaw-tolerance: 0.35 rad
waist-roll-tolerance: 0.20 rad
waist-pitch-tolerance: 0.20 rad
```

保存文件：

```text
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

### 2.7 支持 file 和 ROS2 两种目标输入

文件模式：

```bash
./run/05_plan_only_target.sh
```

读取：

```text
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
```

ROS2 模式：

```bash
G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

订阅：

```text
/g1/locked_grasp_target
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

要求：

```text
header.frame_id == pelvis
```

如果视觉端发来的不是 `pelvis` 坐标系，必须先做 TF 转换，不能直接规划。

### 2.8 增加轨迹审查

新增：

```text
src/g1_grasp_planner/g1_grasp_planner/review_trajectory.py
```

运行：

```bash
./run/06_review_last_trajectory.sh
```

检查内容：

- 轨迹点数量。
- 每个点的维度是否匹配 joint_names。
- `time_from_start` 是否递增。
- position / velocity / acceleration 是否都是有限值。
- 速度是否超过限位。
- 加速度是否超过限位。
- 相邻点隐含速度是否异常。
- 每个关节的 start/end/min/max/delta。
- 腰部 yaw/roll/pitch 是否超过自然动作阈值。

通过输出：

```text
TRAJECTORY_REVIEW_PASSED
```

报告：

```text
runtime/last_trajectory_review.json
```

### 2.8.1 修复异常扭腰轨迹

曾经出现过一种情况：MoveIt 规划成功，但是 Isaac Sim/RViz 里看起来上半身大幅扭转，动作不自然。

原因是：

```text
right_arm 规划组的 chain 从 pelvis 到 right_hand_palm_link
```

所以 `right_arm` 实际包含：

```text
waist_yaw_joint
waist_roll_joint
waist_pitch_joint
right arm joints
```

原始请求只要求末端到达目标点，姿态约束也比较松，MoveIt/OMPL 会选择“能到点”的 IK 解，不会自动保证动作像人一样自然。旧轨迹虽然 plan succeeded，但腰部变化过大：

```text
waist_yaw_joint   max_abs=1.9188 rad
waist_roll_joint  max_abs=0.5200 rad
waist_pitch_joint max_abs=0.5200 rad
```

当前已经在 `moveit_plan_only_node.py` 中加入腰部 goal/path constraints，并在 `run/05_plan_only_target.sh` 默认启用：

```text
waist_yaw_joint   0.0 +/- 0.35 rad
waist_roll_joint  0.0 +/- 0.20 rad
waist_pitch_joint 0.0 +/- 0.20 rad
```

同时 `review_trajectory.py` 会把明显异常的腰部运动判失败。当前修复后的成功轨迹示例：

```text
target_xyz: [0.30, -0.14, 0.07]
trajectory points: 39
duration: 3.721s
waist_yaw_joint   max_abs=0.1149 rad
waist_roll_joint  max_abs=0.0869 rad
waist_pitch_joint max_abs=0.0806 rad
TRAJECTORY_REVIEW_PASSED
```

如果以后某个目标点在腰部受限后规划失败，不要直接放开腰部限制。优先调整抓取策略：

```text
1. 提高 pre-grasp 点。
2. 先到目标上方/前方。
3. 再沿短直线接近目标。
4. 必要时重新检查目标点坐标系和 z 高度。
```

### 2.9 增加执行前总闸门

新增：

```text
src/g1_grasp_planner/g1_grasp_planner/pre_execution_gate.py
```

运行：

```bash
./run/07_pre_execution_gate.sh
```

检查内容：

- 目标点文件存在。
- 目标点文件没有过期。
- 轨迹 JSON 存在。
- 轨迹 JSON 没有过期。
- 审查报告存在并且比轨迹新。
- 审查报告没有 error。
- 当前目标点 + pick offset 和轨迹记录的目标一致。
- MoveIt 当前状态通过 `/check_state_validity`。

通过输出：

```text
PRE_EXECUTION_GATE_PASSED
```

报告：

```text
runtime/pre_execution_gate_report.json
```

### 2.10 增加 Unitree 映射 dry-run

新增映射文件：

```text
config/unitree_g1_29_joint_map.yaml
```

新增 dry-run：

```text
src/g1_grasp_planner/g1_grasp_planner/trajectory_dry_run_bridge.py
```

运行：

```bash
./run/08_dry_run_unitree_bridge.sh
```

它只打印未来执行时会写入的 motor index，不发真机命令。

当前 MoveIt 到 Unitree 映射：

```text
waist_yaw_joint             -> motor_cmd[12]
waist_roll_joint            -> motor_cmd[13]
waist_pitch_joint           -> motor_cmd[14]
right_shoulder_pitch_joint  -> motor_cmd[22]
right_shoulder_roll_joint   -> motor_cmd[23]
right_shoulder_yaw_joint    -> motor_cmd[24]
right_elbow_joint           -> motor_cmd[25]
right_wrist_roll_joint      -> motor_cmd[26]
right_wrist_pitch_joint     -> motor_cmd[27]
right_wrist_yaw_joint       -> motor_cmd[28]
```

`rt/arm_sdk` 权重关节：

```text
kNotUsedJoint0 -> motor_cmd[29]
```

注意：当前仍然不执行真机，只做 dry-run。

### 2.11 增加硬件映射验证

新增：

```text
src/g1_grasp_planner/g1_grasp_planner/verify_hardware_mapping.py
```

运行：

```bash
./run/09_verify_hardware_mapping.sh
```

检查：

- 轨迹里的 joint_names 是否都有 motor index。
- motor index 是否重复。
- weight_joint 是否和控制关节冲突。

通过输出：

```text
HARDWARE_MAPPING_PASSED
```

### 2.12 增加 RViz 轨迹重放

新增：

```text
src/g1_grasp_planner/g1_grasp_planner/replay_trajectory_rviz.py
```

运行：

```bash
./run/10_replay_last_trajectory_rviz.sh
```

作用：

- 读取 `runtime/last_plan_only_trajectory.json`。
- 发布 `moveit_msgs/msg/DisplayTrajectory` 到 `/display_planned_path`。
- 让 RViz 重复显示上一次规划轨迹。

## 3. 每次运行应该怎么操作

### 3.1 清理旧 MoveIt 进程

如果之前启动过多次 `03_demo_rviz.sh`，可能有多个 `/move_action`，会出现：

```text
Ignoring unexpected goal response. There may be more than one action server for the action '/move_action'
```

这种情况先关掉旧终端，必要时清理：

```bash
pkill -f move_group
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f rviz2
```

### 3.2 编译

修改代码或配置后运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
```

成功标准：

```text
Finished <<< g1_moveit_config
Finished <<< g1_grasp_planner
```

### 3.3 启动 MoveIt 和 RViz

终端 1：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

正常会启动：

```text
robot_state_publisher
joint_state_publisher
move_group
rviz2
```

可以忽略的常见 warning：

```text
The root link pelvis has an inertia specified in the URDF
No 3D sensor plugin(s) defined for octomap updates
Stereo is NOT SUPPORTED
```

这些不影响当前 plan-only 验证。

### 3.4 发布静态场景

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
```

正常输出：

```text
Published static planning scene: table_keepout; removed torso_front_keepout
```

### 3.5 文件模式规划

确认目标文件存在：

```bash
cat /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
cat /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
```

运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

默认是 `G1_ARM=auto`，会优先读取 `locked_target_hand.txt`，没有 hand 文件时按目标点 `y` 自动选择。手动覆盖左手：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/05_plan_only_target.sh
```

手动覆盖右手：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=right ./run/05_plan_only_target.sh
```

左手会自动切换为：

```text
group_name: left_arm
end_effector_link: left_hand_palm_link
pick_offset: [0.0, 0.02, 0.02]
```

右手会自动使用：

```text
group_name: right_arm
end_effector_link: right_hand_palm_link
pick_offset: [0.0, -0.02, 0.02]
```

成功输出类似：

```text
Plan succeeded: ... trajectory points, joints=[...]. No execution was requested.
Saved plan-only trajectory: /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

### 3.6 ROS2 模式规划

如果识别端通过 ROS2 发布目标点，运行：

```bash
cd /home/louisxx/g1_moveit_ws
G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

ROS2 模式同样默认 `G1_ARM=auto`。如果要手动覆盖左手：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

它会等待：

```text
/g1/locked_grasp_target
```

测试发布命令示例：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /g1/locked_grasp_target geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'pelvis'}, point: {x: 0.30, y: -0.12, z: 0.05}}"
```

### 3.7 审查轨迹

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

必须看到：

```text
TRAJECTORY_REVIEW_PASSED
```

### 3.8 执行前总闸门

```bash
cd /home/louisxx/g1_moveit_ws
./run/07_pre_execution_gate.sh
```

默认 auto 会重新读取 hand 文件或目标点并匹配上一条轨迹。如果上一条轨迹是手动强制左手规划，闸门也要使用同一个手臂开关：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/07_pre_execution_gate.sh
```

必须看到：

```text
PRE_EXECUTION_GATE_PASSED
```

如果提示目标过期，说明 `locked_target_xyz.txt` 太久没有更新。重新运行识别端生成目标点，或者测试时临时刷新文件时间：

```bash
touch /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

### 3.9 dry-run Unitree 映射

```bash
cd /home/louisxx/g1_moveit_ws
./run/08_dry_run_unitree_bridge.sh
```

默认 auto 会重新读取 hand 文件或目标点并匹配上一条轨迹。如果上一条轨迹是手动强制左手规划，dry-run 也要使用同一个手臂开关：

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/08_dry_run_unitree_bridge.sh
```

必须看到：

```text
DRY_RUN_BRIDGE_READY
```

这一步仍然不发真机命令。

### 3.10 验证硬件映射

```bash
cd /home/louisxx/g1_moveit_ws
./run/09_verify_hardware_mapping.sh
```

必须看到：

```text
HARDWARE_MAPPING_PASSED
```

### 3.11 在 RViz 重放轨迹

如果规划成功但 RViz 没看到轨迹，运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/10_replay_last_trajectory_rviz.sh
```

然后看 RViz 左侧：

```text
Displays -> MotionPlanning -> Planned Path
```

确保 `MotionPlanning` 是勾选状态。

## 4. 推荐完整运行顺序

普通 file 模式：

```bash
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
./run/03_demo_rviz.sh
```

另开一个终端：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
./run/05_plan_only_target.sh
./run/06_review_last_trajectory.sh
./run/07_pre_execution_gate.sh
./run/08_dry_run_unitree_bridge.sh
./run/09_verify_hardware_mapping.sh
```

需要重放轨迹时：

```bash
cd /home/louisxx/g1_moveit_ws
./run/10_replay_last_trajectory_rviz.sh
```

ROS2 目标模式只需要把：

```bash
./run/05_plan_only_target.sh
```

换成：

```bash
G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

## 5. 当前能实现的效果

现在可以实现：

- 加载 G1 机器人模型。
- 启动 MoveIt `move_group`。
- 在 RViz 中显示机器人和规划轨迹。
- 从原视觉 pipeline 的目标点文件读取 3D 目标。
- 从 ROS2 topic 读取 `PointStamped` 目标。
- 默认根据 `locked_target_hand.txt` 或目标点 `y` 自动选择右臂/左臂规划到目标附近。
- 通过 `G1_ARM=left/right` 手动覆盖自动选择。
- 默认限制腰部大幅扭转，避免“数学可行但动作不自然”的轨迹。
- 保存 MoveIt 规划结果为 JSON。
- 离线检查轨迹是否明显越界。
- 离线检查腰部 yaw/roll/pitch 是否异常。
- 检查目标点和轨迹是否匹配。
- 检查 MoveIt 当前状态是否有效。
- 检查 MoveIt 关节名到 Unitree motor index 的映射。
- 在不发真机命令的情况下预览将来会控制哪些电机。

现在还不能实现：

- 真机自动执行抓取。
- 自动闭合手指抓取。
- Isaac Sim 中的接触抓取验证。
- 动态视觉伺服。
- 失败后自动重规划和恢复。
- 对运动过程中的真实碰撞做传感器闭环判断。

## 6. 和原 grasp pipeline 的区别

原 `g1_grasp_pipeline_workspace` 主要解决：

```text
看见目标在哪里
```

它负责：

- RealSense 图像和深度。
- YOLO 检测。
- 目标稳定锁定。
- 像素和深度转 3D。
- 手眼标定。
- 输出 `locked_target_xyz.txt` 或 ROS2 目标点。

当前 `g1_moveit_ws` 主要解决：

```text
机器人手怎么过去
```

它负责：

- 读取目标点。
- 检查坐标系和工作空间。
- 调 MoveIt 规划右臂路径。
- 输出关节轨迹。
- 审查轨迹。
- 为后续仿真和真机执行准备安全闸门。

两者合起来才是：

```text
识别目标 -> 生成目标 3D 点 -> MoveIt 规划路径 -> 仿真验证 -> 真机执行
```

## 7. 常见问题

### 7.1 `joint_state_publisher` 找不到

现象：

```text
package 'joint_state_publisher' not found
```

处理：

```bash
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher
```

### 7.2 OMPL planner 为空

现象：

```text
Planning plugin name is empty or not defined in namespace 'ompl'
```

处理：

- 检查 `config/ompl_planning.yaml` 是否有 `planning_plugins`。
- 重新编译：

```bash
./run/02_build.sh
```

### 7.3 规划失败 `MoveIt error_code=99999`

可能原因：

- `move_group` 没正常启动。
- 目标点在碰撞区。
- 目标点超出工作空间。
- 场景障碍物太粗。
- 同时存在多个 `/move_action`。
- 目标坐标系不是 `pelvis`。

建议处理顺序：

```bash
pkill -f move_group
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f rviz2
```

然后重新启动：

```bash
./run/03_demo_rviz.sh
./run/04_static_scene.sh
./run/05_plan_only_target.sh
```

### 7.4 规划成功但 RViz 看不到轨迹

处理：

```bash
./run/10_replay_last_trajectory_rviz.sh
```

同时确认 RViz 中：

```text
MotionPlanning -> Planned Path
```

已启用。

### 7.6 规划成功但上半身异常扭转

先运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

看 `joint_stats` 中的腰部关节：

```text
waist_yaw_joint
waist_roll_joint
waist_pitch_joint
```

如果腰部超过阈值，当前脚本会输出：

```text
TRAJECTORY_REVIEW_FAILED
```

处理顺序：

```text
1. 确认 run/05_plan_only_target.sh 已带 --constrain-waist 和 --waist-path-constraints。
2. 重新 build：./run/02_build.sh。
3. 重启 MoveIt/RViz。
4. 重新发布 static scene。
5. 重新规划和审查。
6. 如果仍失败，检查目标点是否太低或太靠近身体。
```

### 7.7 左手规划失败

左手链路已经接入，但目标点必须在左手合适工作区。右侧目标点直接切左手通常会失败。

检查日志里的请求：

```text
group=left_arm
link=left_hand_palm_link
```

如果日志里出现：

```text
No acceleration limit was defined for joint left_shoulder_pitch_joint
```

说明 MoveIt 没加载到更新后的 `joint_limits.yaml`。处理：

```bash
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
pkill -f "ros2 launch g1_moveit_config demo.launch.py"
./run/03_demo_rviz.sh
```

当前已补齐左臂 7 个 arm joint 的 acceleration limit。一个已知可规划的左手测试目标是：

```text
locked target: [0.35, 0.20, 0.18]
left pick target: [0.35, 0.22, 0.20]
result: 23 trajectory points, TRAJECTORY_REVIEW_PASSED
```

### 7.5 总闸门说目标过期

说明目标文件太久没有更新。正常做法是重新运行识别端。测试时可以：

```bash
touch /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

## 8. 从 0 复现应该怎么做

如果你自己从头创建并跑通这个项目，按下面顺序：

1. 安装 ROS2 Jazzy 和 MoveIt 相关包。
2. 创建 `/home/louisxx/g1_moveit_ws`。
3. 放入 G1 URDF 和 meshes。
4. 用 MoveIt Setup Assistant 生成 `g1_moveit_config`。
5. 配置 `right_arm`、`left_arm`、`dual_arm` 规划组。
6. 检查 SRDF 自碰撞矩阵。
7. 配置 `kinematics.yaml`。
8. 配置 `joint_limits.yaml`，特别是速度和加速度限制。
9. 配置 `ompl_planning.yaml`，确保有 `ompl_interface/OMPLPlanner`。
10. 配置 RViz，让它订阅 `/display_planned_path`。
11. 写 `static_scene_publisher.py`，给 MoveIt planning scene 发布桌面。
12. 写 `moveit_plan_only_node.py`，从 file 或 ROS2 topic 读取目标点。
13. 只发送 `plan_only=true` 的 MoveIt 请求。
14. 保存 `JointTrajectory` 到 JSON。
15. 写轨迹审查脚本。
16. 写执行前总闸门。
17. 写 Unitree motor index 映射文件。
18. 写 dry-run bridge，只打印不执行。
19. 写 RViz replay 脚本，方便重复看轨迹。
20. 先在 RViz 验证，再接 Isaac Sim，最后才考虑真机。

## 9. 下一步建议

下一步不要直接接真机。建议顺序是：

```text
MoveIt/RViz 继续稳定
  -> Isaac Sim 播放 last_plan_only_trajectory.json
  -> Isaac Sim 加桌子和目标物体
  -> Isaac Sim 验证接触抓取
  -> Unitree 执行桥继续 dry-run
  -> 小幅度真机空载测试
  -> 真机抓取测试
```

当前已经补了 Isaac Sim 入口文档：

```text
ISAAC_SIM_RUNBOOK.md
```

当前 Isaac Sim Docker 镜像已经可以启动 GUI，并且可以通过 `G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh` 播放最新 MoveIt 轨迹。

自适应抓取的仿真入口是 `./run/20_isaacsim_gui_adaptive_grasp.sh`。
