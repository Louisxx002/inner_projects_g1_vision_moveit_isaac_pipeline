# G1 MoveIt 规划工作区运行与复现文档

本文档记录 `/home/louisxx/g1_moveit_ws` 当前已经打通的能力、运行步骤、与原有
`g1_grasp_pipeline_workspace` 的关系，以及后续接仿真和真机执行的路线。

## 1. 整个运行操作和对应效果

当前工作区实现的是：

```text
目标点文件
  -> MoveIt plan-only 路径规划
  -> RViz 可视化轨迹
  -> 保存轨迹 JSON
  -> 离线轨迹审查
  -> 执行前总闸门
  -> Unitree motor_cmd 映射 dry-run
```

它不会向真机发送控制命令。

### 1.1 启动 MoveIt + RViz

终端 1：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

效果：

- 加载 G1 URDF/SRDF。
- 启动 `robot_state_publisher`。
- 启动 `joint_state_publisher`。
- 启动 `move_group`。
- 启动 RViz，并加载 `MotionPlanning` 插件。
- RViz 订阅 `/display_planned_path`，可以显示规划轨迹。

看到下面日志说明 MoveIt 已经可规划：

```text
You can start planning now!
```

### 1.2 发布静态场景

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
```

效果：

- 向 MoveIt planning scene 发布桌面碰撞体 `table_keepout`。
- 移除之前过于粗糙、会误碰手腕的 `torso_front_keepout`。

当前输出：

```text
Published static planning scene: table_keepout; removed torso_front_keepout
```

### 1.3 从目标点规划路径

目标点来自：

```bash
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
```

运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

效果：

- 读取目标点。
- 默认 `G1_ARM=auto`，优先按 `locked_target_hand.txt` 自动选择左手或右手；没有 hand 文件时按目标点 `y` 值判断。
- 根据选中的手加 pick offset：

```text
right: [0.0, -0.02, 0.02]
left:  [0.0, 0.02, 0.02]
```

- 对 `right_arm` 或 `left_arm` 规划。
- 末端 link 是 `right_hand_palm_link` 或 `left_hand_palm_link`。
- 只规划，不执行。
- 保存轨迹到：

```bash
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

成功时会看到：

```text
Plan succeeded: ... trajectory points
No execution was requested.
```

### 1.4 在 RViz 里重放轨迹

如果只跑 `05_plan_only_target.sh` 没看清轨迹，可以运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/10_replay_last_trajectory_rviz.sh
```

效果：

- 读取 `runtime/last_plan_only_trajectory.json`。
- 连续 20 秒向 `/display_planned_path` 发布轨迹。
- RViz 的 `MotionPlanning -> Planned Path` 会重复显示轨迹。

如果看不到，检查 RViz 左侧：

```text
Displays
  MotionPlanning
    Planned Path
```

确保 `MotionPlanning` 被勾选。

### 1.5 审查轨迹

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

效果：

- 检查轨迹点数量。
- 检查时间严格递增。
- 检查位置、速度、加速度都是有限值。
- 检查声明速度不超过 `joint_limits.yaml * velocity_scale`。
- 检查声明加速度不超过 `joint_limits.yaml * acceleration_scale`。
- 检查相邻轨迹点的隐含速度。

通过时：

```text
TRAJECTORY_REVIEW_PASSED
```

报告保存到：

```bash
/home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json
```

### 1.6 执行前总闸门

```bash
cd /home/louisxx/g1_moveit_ws
./run/07_pre_execution_gate.sh
```

效果：

- 检查目标点文件存在且未过期。
- 检查轨迹文件存在且未过期。
- 检查轨迹审查报告存在且比轨迹更新。
- 检查审查报告没有错误。
- 检查当前目标点 + pick offset 和轨迹里的目标点一致。
- 调用 MoveIt `/check_state_validity` 检查当前状态无碰撞。

通过时：

```text
PRE_EXECUTION_GATE_PASSED
```

报告保存到：

```bash
/home/louisxx/g1_moveit_ws/runtime/pre_execution_gate_report.json
```

### 1.7 Unitree dry-run 桥接

```bash
cd /home/louisxx/g1_moveit_ws
./run/08_dry_run_unitree_bridge.sh
```

效果：

- 先执行总闸门检查。
- 检查通过后，打印未来如果执行，会写入哪些 `motor_cmd[index]`。
- 不初始化 DDS。
- 不连接 Unitree。
- 不发送真机命令。

当前映射：

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

`rt/arm_sdk` 权重开关：

```text
kNotUsedJoint0 -> motor_cmd[29]
```

### 1.8 硬件映射验证

```bash
cd /home/louisxx/g1_moveit_ws
./run/09_verify_hardware_mapping.sh
```

效果：

- 检查 MoveIt 轨迹里的关节名是否都能映射到 Unitree G1 29DoF 的 motor index。
- 检查映射是否重复。
- 检查 `weight_joint` 是否和控制关节冲突。

通过时：

```text
HARDWARE_MAPPING_PASSED
```

## 2. 后续可以用 Isaac Gym / Isaac Sim 仿真吗

可以接仿真，但建议路线不是旧 Isaac Gym，而是 Isaac Sim / Isaac Lab。

NVIDIA 官方页面说明 Isaac Gym 现在是 legacy/deprecated 软件，不再支持；官方建议使用 Isaac Lab，它构建在 Isaac Sim 平台上。Isaac Sim 支持机器人仿真、传感器仿真、ROS 工作流、物理仿真和合成数据生成；Isaac Lab 是面向机器人学习、强化学习、模仿学习的框架。

推荐路线：

```text
当前 RViz/MoveIt
  -> Isaac Sim 导入 G1 URDF/USD
  -> 建立桌子、目标物体、相机
  -> 用 ROS2 或脚本把 MoveIt 轨迹送入 Isaac Sim 控制器
  -> 在 Isaac Sim 里验证轨迹、碰撞、接触、抓取
  -> 稳定后再考虑真机 rt/arm_sdk
```

如果只是看轨迹：

```text
RViz 足够
```

如果要看物理接触、桌子碰撞、物体是否被抓起：

```text
Isaac Sim 更合适
```

如果要训练策略、做大规模并行强化学习：

```text
Isaac Lab 更合适
```

旧 Isaac Gym 仍可用于一些已有代码，但不建议作为新工程基础。

参考：

- NVIDIA Isaac Gym 页面说明其为 legacy/deprecated，并建议使用 Isaac Lab。
- NVIDIA Isaac Sim 是面向机器人仿真和合成数据的 Omniverse 平台。
- NVIDIA Isaac Lab 是 Isaac Sim 上的官方机器人学习框架。

## 3. 后续怎么打通识别、自动规划、抓取路径

当前已有两个工作区：

```text
/home/louisxx/g1_grasp_pipeline_workspace
/home/louisxx/g1_moveit_ws
```

它们应该按下面方式连接。

注意：这两个工作区运行在不同环境里。

```text
g1_grasp_pipeline_workspace:
  视觉脚本使用 /home/louisxx/miniconda3/envs/g1_vision/bin/python

g1_moveit_ws:
  MoveIt 脚本使用 ROS 2 Jazzy + 当前 colcon install overlay
```

正常手动运行时不需要自己 `conda activate` 或手动 `source`，因为对应的 `run/*.sh`
已经处理了环境：

```text
01_run_vision_file.sh:
  source config/paths.env
  调用 VISION_PYTHON=/home/louisxx/miniconda3/envs/g1_vision/bin/python

05_plan_only_target.sh:
  source /opt/ros/jazzy/setup.bash
  source /home/louisxx/g1_moveit_ws/install/setup.bash
```

也就是说，需要管环境边界，但不要在同一个 shell 里混着手动激活；按脚本入口跑即可。

### 3.1 识别端

已有 `g1_grasp_pipeline_workspace` 做：

```text
RealSense + YOLO
  -> 稳定检测目标
  -> 深度点转 3D 点
  -> 手眼标定转到 pelvis 坐标系
  -> 写入 locked_target_xyz.txt
```

运行：

file 模式：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/01_run_vision_file.sh
```

这个脚本会使用 `g1_vision` conda 环境，不依赖当前终端是否已经 `conda activate`。

输出：

```bash
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

ROS2 模式：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/03_run_vision_ros2.sh
```

这个脚本会：

```text
source /opt/ros/jazzy/setup.bash
调用 g1_vision Python
同时写 locked_target_xyz.txt
并发布 /g1/locked_grasp_target
```

ROS2 topic：

```text
/g1/locked_grasp_target
geometry_msgs/msg/PointStamped
frame_id="pelvis"
```

### 3.2 自动规划端

MoveIt 工作区现在支持两种输入。

file 模式，读取同一个目标点文件：

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

这个脚本会自己 source ROS 2 Jazzy 和 `g1_moveit_ws/install/setup.bash`。

输出：

```bash
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

ROS2 模式，订阅 `PointStamped`：

```bash
cd /home/louisxx/g1_moveit_ws
G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

这个脚本会等待：

```text
/g1/locked_grasp_target
```

收到目标后进行同样的 MoveIt plan-only 规划，并保存同一个轨迹 JSON：

```bash
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

因此现在两套链路都可以用：

```text
file:
  01_run_vision_file.sh -> 05_plan_only_target.sh

ROS2:
  03_run_vision_ros2.sh -> G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

推荐实际操作时用多个终端：

```text
终端 1: g1_moveit_ws/03_demo_rviz.sh
终端 2: g1_grasp_pipeline_workspace/01_run_vision_file.sh
终端 3: g1_moveit_ws/05_plan_only_target.sh、06、07、08、09、10
```

两个工作区通过文件桥连接：

```text
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

所以它们不需要处在同一个 Python/ROS 环境里，只要这个文件路径一致即可。

### 3.3 审查与闸门

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
./run/07_pre_execution_gate.sh
./run/08_dry_run_unitree_bridge.sh
```

### 3.4 未来完整自动链路

未来可以写一个 orchestration 脚本，比如：

```text
1. 等待视觉端锁定目标
2. 检查目标点新鲜度
3. 调用 MoveIt plan-only
4. 保存轨迹
5. 审查轨迹
6. 执行前总闸门
7. dry-run 输出 motor_cmd 映射
8. 人工确认
9. 真执行桥发送 rt/arm_sdk
10. 夹爪/灵巧手闭合
11. 抬起/放置
```

在接真机前必须继续加：

```text
目标过期 -> 不动
规划失败 -> 不动
审查失败 -> 不动
当前状态碰撞 -> 不动
关节限位异常 -> 不动
速度/加速度异常 -> 不动
DDS 超时 -> 停止
人工急停 -> 停止
```

## 4. 这个和原先 grasp pipeline 的区别

原先 `g1_grasp_pipeline_workspace` 更偏向：

```text
视觉锁点
  -> IK 求解
  -> g1-primitives 抓取模板
  -> Unitree SDK2 / Inspire 手
```

它的重点是完整抓取流程和真机控制模板。

现在 `g1_moveit_ws` 更偏向：

```text
MoveIt 规划
  -> 碰撞场景
  -> 轨迹可视化
  -> 轨迹保存
  -> 轨迹审查
  -> 执行前安全闸门
  -> Unitree motor index 映射
```

两者区别：

| 项目 | 原 grasp pipeline | g1_moveit_ws |
|---|---|---|
| 目标 | 视觉抓取闭环 | 规划与安全验证 |
| 输入 | RealSense/YOLO 目标点 | 目标点文件 |
| 规划方式 | IK + 模板动作 | MoveIt/OMPL 轨迹规划 |
| 碰撞检查 | 较弱，主要靠模板安全 | MoveIt planning scene |
| 可视化 | 主要命令行/实际动作 | RViz 显示轨迹 |
| 输出 | 真机控制流程 | plan-only 轨迹 JSON |
| 真机控制 | 已有模板 | 当前只 dry-run |
| 安全层 | 基础 dry-run | 轨迹审查 + 总闸门 |

可以理解为：

```text
原 grasp pipeline 负责“看见目标并完成抓取流程”
g1_moveit_ws 负责“在动之前先规划、检查、可视化、审查”
```

最终理想组合：

```text
grasp pipeline 的视觉锁点
  -> g1_moveit_ws 的 MoveIt 安全规划
  -> 通过总闸门
  -> Unitree 执行桥
  -> 原 pipeline 的手部抓取/放置逻辑
```

## 5. 如果从零开始创建并跑通这个项目，需要做什么

### 5.1 安装 ROS2 Jazzy 和 MoveIt

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-xacro \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher
```

验证：

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_setup_assistant
ros2 pkg prefix joint_state_publisher
```

### 5.2 创建工作区

```bash
mkdir -p /home/louisxx/g1_moveit_ws/src
cd /home/louisxx/g1_moveit_ws
```

准备：

```text
src/g1_moveit_config
src/g1_grasp_planner
```

### 5.3 准备 G1 URDF 和 meshes

从现有 G1 assets 拷贝：

```text
g1_body29_hand14.urdf
meshes/
```

放入 MoveIt config package。

### 5.4 用 MoveIt Setup Assistant 生成 SRDF

```bash
cd /home/louisxx/g1_moveit_ws
./run/01_setup_assistant.sh
```

规划组：

```text
right_arm: pelvis -> right_hand_palm_link
left_arm:  pelvis -> left_hand_palm_link
dual_arm:  左右手臂关节组合
```

### 5.5 修 OMPL 配置

`config/ompl_planning.yaml` 必须包含：

```yaml
planning_plugins:
  - ompl_interface/OMPLPlanner
request_adapters:
  - default_planning_request_adapters/ResolveConstraintFrames
  - default_planning_request_adapters/ValidateWorkspaceBounds
  - default_planning_request_adapters/CheckStartStateBounds
  - default_planning_request_adapters/CheckStartStateCollision
response_adapters:
  - default_planning_response_adapters/AddTimeOptimalParameterization
  - default_planning_response_adapters/ValidateSolution
  - default_planning_response_adapters/DisplayMotionPath
```

否则会出现：

```text
Planning plugin name is empty or not defined in namespace 'ompl'
```

### 5.6 修 SRDF 自碰撞

需要为相邻 link 添加 `disable_collisions`，否则零位状态会一开始就碰撞，规划失败。

重点包括：

```text
pelvis <-> waist_yaw_link
torso_link <-> head_link
torso_link <-> logo_link
左右手掌、手指、腕部相邻 link
肩肘腕相邻 link
```

### 5.7 修 joint_limits 加速度

MoveIt 的时间参数化需要 acceleration limits。

至少给当前规划链路里的关节加：

```text
waist_yaw_joint
waist_roll_joint
waist_pitch_joint
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
```

否则会出现：

```text
No acceleration limit was defined for joint waist_yaw_joint
AddTimeOptimalParameterization failed
```

### 5.8 写 plan-only 节点

节点做：

```text
读取 locked_target_xyz.txt
加 pick offset
构造 MoveGroup action request
设置 plan_only=True
保存 planned_trajectory 到 JSON
```

### 5.9 写轨迹审查脚本

检查：

```text
时间递增
关节名一致
点维度一致
位置/速度/加速度有限
速度限位
加速度限位
点间隐含速度
```

### 5.10 写执行前总闸门

检查：

```text
目标文件新鲜
轨迹文件新鲜
审查报告新鲜
目标点和轨迹目标一致
MoveIt 当前状态有效
```

### 5.11 写 dry-run 执行桥

先只打印：

```text
MoveIt joint -> Unitree motor_cmd[index]
每个轨迹点时间
每个轨迹点 q
```

不发 DDS。

### 5.12 验证硬件映射

G1 29DoF 当前映射：

```text
waist_yaw_joint             -> 12
waist_roll_joint            -> 13
waist_pitch_joint           -> 14
right_shoulder_pitch_joint  -> 22
right_shoulder_roll_joint   -> 23
right_shoulder_yaw_joint    -> 24
right_elbow_joint           -> 25
right_wrist_roll_joint      -> 26
right_wrist_pitch_joint     -> 27
right_wrist_yaw_joint       -> 28
```

权重开关：

```text
motor_cmd[29]
```

### 5.13 一次完整验证

```bash
cd /home/louisxx/g1_moveit_ws
./run/02_build.sh
./run/03_demo_rviz.sh
```

新终端：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
./run/05_plan_only_target.sh
./run/10_replay_last_trajectory_rviz.sh
./run/06_review_last_trajectory.sh
./run/07_pre_execution_gate.sh
./run/08_dry_run_unitree_bridge.sh
./run/09_verify_hardware_mapping.sh
```

全部通过后，才算“规划和执行前检查层”跑通。

## 6. 当前还能做什么，不能做什么

### 已经能做

```text
读取视觉目标点
MoveIt 规划右臂/腰部轨迹
RViz 显示规划轨迹
保存轨迹 JSON
审查轨迹安全性
执行前总闸门
验证 Unitree motor index 映射
打印 dry-run motor_cmd
```

### 还不能做

```text
不能直接发 rt/arm_sdk
不能让真机动
不能完成物理抓取仿真
不能保证目标物体接触/摩擦/抓取稳定
不能替代人工急停
```

### 下一步建议

优先级从高到低：

```text
1. 在 RViz 中多测不同目标点
2. 增加更精确的躯干/桌面/禁区碰撞体
3. 接 Isaac Sim 做物理仿真
4. 写 Unitree DDS 真执行桥，但默认锁死为 disabled
5. 增加人工确认和急停输入
6. 最后接真机
```
