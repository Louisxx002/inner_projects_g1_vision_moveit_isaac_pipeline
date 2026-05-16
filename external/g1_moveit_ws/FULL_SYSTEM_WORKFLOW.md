# G1 识别-规划-仿真-抓取完整工作流程

本文档记录当前 G1 抓取项目从视觉识别、目标点传递、MoveIt 规划、Isaac Sim 仿真，到后续真机执行的完整工程流程。

涉及两个主要工作区：

```text
/home/louisxx/g1_grasp_pipeline_workspace
/home/louisxx/g1_moveit_ws
```

当前状态：

```text
视觉端 grasp pipeline 已能锁定目标点并通过 file/ROS2 输出。
MoveIt 端已能读取目标点和 locked_target_hand.txt，默认 auto 自动选择左臂或右臂轨迹。
MoveIt 端已加入腰部约束，避免规划时出现异常上半身扭转。
Isaac Sim 端已能用 Docker GUI 打开并播放最新 MoveIt 轨迹。
真机执行桥尚未打开，当前不会向 Unitree 发送运动命令。
```

## 1. 总体目标

最终目标是：

```text
相机看见目标
  -> YOLO 检测目标
  -> 深度图得到目标 3D 点
  -> 手眼标定转换到 pelvis 坐标系
  -> MoveIt 根据目标侧自动规划 G1 左臂或右臂路径
  -> Isaac Sim 中验证轨迹和抓取
  -> 通过安全闸门后再接 Unitree 真机执行
```

工程上分成四层：

```text
1. grasp pipeline：负责“看见目标在哪里”
2. MoveIt workspace：负责“手臂怎么过去”
3. Isaac Sim：负责“先在物理仿真里验证轨迹和接触”
4. Unitree bridge：负责“最后才把轨迹变成真机 motor_cmd”
```

## 2. 当前目录分工

### 2.1 视觉/原始抓取工作区

路径：

```text
/home/louisxx/g1_grasp_pipeline_workspace
```

它负责：

- RealSense RGB-D 采集。
- YOLO 目标检测。
- 连续多帧稳定后锁定目标。
- 深度点转相机坐标 3D 点。
- 通过 `T_pelvis_camera.npy` 转到 G1 `pelvis` 坐标系。
- 通过 file 或 ROS2 输出锁定目标。
- 通过 `runtime/locked_target_hand.txt` 输出自动推断的抓取侧。
- 原始抓取端可直接调用 g1-primitives / Unitree SDK2。

重要文件：

```text
demo_realsense_realtime_lock_target.py
examples/ros2_locked_target_usage.py
examples/basic_usage.py
config/paths.env
calibration/T_pelvis_camera.npy
runtime/locked_target_xyz.txt
runtime/locked_target_hand.txt
run/01_run_vision_file.sh
run/03_run_vision_ros2.sh
```

### 2.2 MoveIt 规划工作区

路径：

```text
/home/louisxx/g1_moveit_ws
```

它负责：

- 加载 G1 URDF/SRDF。
- 启动 MoveIt `move_group`。
- 读取 grasp pipeline 输出的目标点。
- 根据 `locked_target_hand.txt` 或目标点 `y` 值自动选择 `right_arm` / `left_arm`。
- 规划到 `right_hand_palm_link` 或 `left_hand_palm_link`。
- 保存 plan-only 轨迹 JSON。
- 在 RViz 显示和重放轨迹。
- 检查速度、加速度、目标新鲜度、碰撞状态。
- 预检查 MoveIt joint 到 Unitree motor index 的映射。

重要文件：

```text
src/g1_moveit_config/config/g1.urdf
src/g1_moveit_config/config/g1.srdf
src/g1_moveit_config/config/ompl_planning.yaml
src/g1_moveit_config/config/joint_limits.yaml
src/g1_grasp_planner/g1_grasp_planner/moveit_plan_only_node.py
src/g1_grasp_planner/g1_grasp_planner/review_trajectory.py
src/g1_grasp_planner/g1_grasp_planner/pre_execution_gate.py
runtime/last_plan_only_trajectory.json
```

### 2.3 Isaac Sim 仿真入口

路径：

```text
/home/louisxx/g1_moveit_ws/isaacsim
```

它负责后续仿真验证：

- 启动 Isaac Sim 容器。
- 导入 G1 URDF 为 USD。
- 读取 MoveIt 保存的轨迹 JSON。
- 在 Isaac Sim 中播放腰部和左/右臂轨迹。
- 后续加桌子、物体、相机、接触抓取。

重要文件：

```text
run/11_isaacsim_container_bash.sh
run/12_check_isaacsim_playback_inputs.sh
isaacsim/play_last_trajectory.py
isaacsim/check_playback_inputs.py
ISAAC_SIM_RUNBOOK.md
```

## 3. 数据怎么流动

### 3.1 file 模式

file 模式最简单、最稳定，适合当前阶段。

```text
RealSense + YOLO
  -> demo_realsense_realtime_lock_target.py
  -> /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
  -> /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt
  -> /home/louisxx/g1_moveit_ws/run/05_plan_only_target.sh
  -> MoveIt /move_action plan-only
  -> /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
  -> RViz / Isaac Sim / dry-run
```

目标文件格式：

```text
x y z
```

示例：

```text
0.30 -0.12 0.05
```

抓取侧辅助文件格式：

```text
right
```

`right` / `left` 来自 pelvis 坐标系 `y` 值判断。中线附近会输出 `center`，此时需要手动指定 `G1_ARM=left` 或 `G1_ARM=right`。

坐标系要求：

```text
pelvis
```

### 3.2 ROS2 模式

ROS2 模式适合后续自动触发和更强的时序管理。

```text
RealSense + YOLO
  -> demo_realsense_realtime_lock_target.py
  -> /g1/locked_grasp_target
  -> geometry_msgs/msg/PointStamped
  -> G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
  -> MoveIt plan-only
```

topic：

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

如果 frame 不是 `pelvis`，必须先做 TF 转换，不能直接送给 MoveIt。

## 4. 从 0 到当前状态做了什么

### 4.1 grasp pipeline 已完成的工作

在 `/home/louisxx/g1_grasp_pipeline_workspace` 中已经整理了：

- 可移植 third_party 依赖。
- RealSense + YOLO 视觉锁点脚本。
- 手眼标定文件 `T_pelvis_camera.npy`。
- file bridge：`runtime/locked_target_xyz.txt`。
- hand sidecar：`runtime/locked_target_hand.txt`。
- ROS2 bridge：`/g1/locked_grasp_target`。
- file 模式抓取入口。
- ROS2 模式抓取入口。
- dry-run 检查脚本。
- portable workspace 检查脚本。
- full safety check 脚本。

常用脚本：

```text
run/00_check_portable_workspace.sh
run/00_full_safety_check.sh
run/00_check_ros2_transport.sh
run/00_dry_run_grasp.sh
run/01_run_vision_file.sh
run/02_run_grasp_file.sh
run/03_run_vision_ros2.sh
run/04_run_grasp_ros2.sh
```

### 4.2 MoveIt 已完成的工作

在 `/home/louisxx/g1_moveit_ws` 中已经完成：

- 创建独立 ROS2 Jazzy / MoveIt 工作区。
- 生成并修复 G1 MoveIt config。
- 修复 `joint_state_publisher` 缺包问题。
- 修复 OMPL planner 配置为空的问题。
- 配置 RViz MotionPlanning 显示。
- 增加静态场景发布。
- 增加 file/ROS2 目标点规划节点。
- 增加轨迹 JSON 保存。
- 增加 RViz 轨迹重放。
- 增加轨迹审查。
- 增加执行前总闸门。
- 增加 Unitree G1 29DoF motor index 映射。
- 增加 dry-run bridge。
- 增加硬件映射验证。

详细 MoveIt 单段文档见：

```text
MOVEIT_WORKFLOW.md
```

### 4.3 Isaac Sim 已完成的工作

在 `/home/louisxx/g1_moveit_ws` 中已经准备：

- Isaac Sim 容器启动脚本。
- 本机输入检查脚本。
- Isaac Sim 内轨迹播放脚本。
- Isaac Sim 操作 runbook。

当前检查结果：

```text
GPU 和 Docker 存在。
nvcr.io/nvidia/isaac-sim:5.1.0 已拉取。
NVIDIA Container Toolkit 已安装并配置。
Isaac Sim GUI 可以启动。
G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh 可以加载 G1 USD 并播放轨迹。
```

当前 Isaac Sim 仍是轨迹回放验证阶段，还不是完整物理抓取闭环。

## 5. 推荐完整运行流程

### 5.1 第一步：检查 grasp pipeline

终端：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/00_check_portable_workspace.sh
./run/00_dry_run_grasp.sh
```

如果要做更完整检查：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/00_full_safety_check.sh
```

如果要验证 ROS2 点传输：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/00_check_ros2_transport.sh
```

期望：

```text
portable workspace check complete
Dry-run mode complete.
FULL SAFETY CHECK PASSED
ROS2_MODE_TRANSPORT_OK
```

### 5.2 第二步：启动视觉锁点

推荐先用 file 模式。

终端 1：

```bash
cd /home/louisxx/g1_grasp_pipeline_workspace
./run/01_run_vision_file.sh
```

它会：

- 打开 RealSense。
- 加载 YOLO 模型。
- 检测目标。
- 等目标连续稳定。
- 写入：

```text
/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

可以另开终端查看：

```bash
cat /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

### 5.3 第三步：启动 MoveIt 和 RViz

终端 2：

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

正常启动：

```text
robot_state_publisher
joint_state_publisher
move_group
rviz2
```

如果之前启动过多次，先清理旧进程：

```bash
pkill -f move_group
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f rviz2
```

### 5.4 第四步：发布 MoveIt 静态场景

终端 3：

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
```

期望输出：

```text
Published static planning scene: table_keepout; removed torso_front_keepout
```

### 5.5 第五步：让 MoveIt 从视觉目标点规划

file 模式：

```bash
cd /home/louisxx/g1_moveit_ws
./run/05_plan_only_target.sh
```

成功时：

```text
Plan succeeded: ... trajectory points
No execution was requested.
Saved plan-only trajectory: /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

ROS2 模式可替换为：

```bash
cd /home/louisxx/g1_moveit_ws
G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
```

### 5.6 第六步：查看 RViz 轨迹

如果 RViz 没自动显示或没看清，重放：

```bash
cd /home/louisxx/g1_moveit_ws
./run/10_replay_last_trajectory_rviz.sh
```

RViz 中检查：

```text
Displays -> MotionPlanning -> Planned Path
```

确保 `MotionPlanning` 被勾选。

### 5.7 第七步：轨迹审查

```bash
cd /home/louisxx/g1_moveit_ws
./run/06_review_last_trajectory.sh
```

必须看到：

```text
TRAJECTORY_REVIEW_PASSED
```

同时看 `joint_stats`。当前腰部受限后的参考结果：

```text
waist_yaw_joint   max_abs ~= 0.1149 rad
waist_roll_joint  max_abs ~= 0.0869 rad
waist_pitch_joint max_abs ~= 0.0806 rad
```

如果看到类似下面的结果，说明轨迹虽然可能规划成功，但动作不自然，不能继续用于仿真/真机：

```text
waist_yaw_joint   max_abs ~= 1.9188 rad
waist_roll_joint  max_abs ~= 0.5200 rad
waist_pitch_joint max_abs ~= 0.5200 rad
TRAJECTORY_REVIEW_FAILED
```

### 5.8 第八步：执行前总闸门

```bash
cd /home/louisxx/g1_moveit_ws
./run/07_pre_execution_gate.sh
```

必须看到：

```text
PRE_EXECUTION_GATE_PASSED
```

如果提示目标点过期，重新运行视觉锁点，或测试时临时刷新：

```bash
touch /home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_xyz.txt
```

### 5.9 第九步：Unitree 映射 dry-run

```bash
cd /home/louisxx/g1_moveit_ws
./run/08_dry_run_unitree_bridge.sh
./run/09_verify_hardware_mapping.sh
```

必须看到：

```text
DRY_RUN_BRIDGE_READY
HARDWARE_MAPPING_PASSED
```

这一步仍然不会发真机命令。

### 5.10 第十步：Isaac Sim 仿真播放

先检查输入：

```bash
cd /home/louisxx/g1_moveit_ws
./run/12_check_isaacsim_playback_inputs.sh
```

如果 Isaac Sim 镜像还没有，网络正常后拉取：

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

启动容器：

```bash
cd /home/louisxx/g1_moveit_ws
./run/11_isaacsim_container_bash.sh
```

容器内把 G1 URDF 导入 Isaac Sim：

```text
/workspace/g1_moveit_ws/src/g1_moveit_config/config/g1.urdf
```

保存为：

```text
/workspace/g1_moveit_ws/runtime/isaac/g1.usd
```

然后播放 MoveIt 轨迹：

```bash
./python.sh /workspace/g1_moveit_ws/isaacsim/play_last_trajectory.py \
  --robot-usd /workspace/g1_moveit_ws/runtime/isaac/g1.usd
```

或者直接在宿主机运行：

```bash
cd /home/louisxx/g1_moveit_ws
./run/15_isaacsim_play_last_trajectory.sh --headless
```

如果要显示 GUI：

```bash
xhost +local:docker
cd /home/louisxx/g1_moveit_ws
./run/17_isaacsim_gui_app.sh
```

如果要打开 GUI 后自动播放 MoveIt 轨迹：

```bash
xhost +local:docker
cd /home/louisxx/g1_moveit_ws
G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
```

当前推荐用这一条看效果。成功时终端会出现：

```text
Simulation App Startup Complete
Playing trajectory: 39 points, joints=[...]
Trajectory playback complete. Isaac Sim will stay open.
```

headless 模式：

```bash
./python.sh /workspace/g1_moveit_ws/isaacsim/play_last_trajectory.py \
  --robot-usd /workspace/g1_moveit_ws/runtime/isaac/g1.usd \
  --headless
```

## 6. 两种总链路运行方式

### 6.1 当前推荐：grasp file -> MoveIt -> Isaac Sim

```text
run/01_run_vision_file.sh
  -> locked_target_xyz.txt
  -> g1_moveit_ws/run/05_plan_only_target.sh
  -> last_plan_only_trajectory.json
  -> RViz / Isaac Sim
```

优点：

- 环境隔离最简单。
- 视觉 conda 环境和 ROS2 MoveIt 环境互不影响。
- 便于调试和复现。
- 文件读取延迟是毫秒级，不是当前瓶颈。

适合：

- 静态目标。
- 锁定后规划。
- 当前阶段的验证。

### 6.2 后续可选：grasp ROS2 -> MoveIt -> Isaac Sim

```text
run/03_run_vision_ros2.sh
  -> /g1/locked_grasp_target
  -> G1_TARGET_SOURCE=ros2 ./run/05_plan_only_target.sh
  -> last_plan_only_trajectory.json
  -> RViz / Isaac Sim
```

优点：

- 更适合自动触发。
- 消息自带 frame_id。
- 后续可以加 timestamp、新鲜度和状态机。

适合：

- 自动化系统。
- 多节点组合。
- 后续闭环抓取。

## 7. 当前能实现什么效果

现在完整项目库能实现：

- 用 RealSense + YOLO 识别目标。
- 锁定稳定目标点。
- 把目标点转换到 pelvis 坐标系。
- 通过 file 或 ROS2 把目标点交给 MoveIt。
- MoveIt 默认对 G1 右臂做 plan-only 路径规划。
- 默认 `G1_ARM=auto` 自动选择左臂或右臂；也可以通过 `G1_ARM=left/right` 手动覆盖。
- 默认限制腰部 yaw/roll/pitch 大幅运动，避免不自然扭腰轨迹。
- 在 RViz 中看机器人和规划路径。
- 把规划结果保存为 JSON。
- 审查轨迹速度、加速度、时间序列和维度。
- 审查腰部是否异常大幅扭转。
- 检查目标点是否过期。
- 检查轨迹目标和当前目标是否一致。
- 检查 MoveIt 当前状态是否碰撞。
- 验证 MoveIt 关节名到 Unitree motor index 的映射。
- 为 Isaac Sim 播放轨迹准备输入。
- 在 Isaac Sim GUI 中播放最新 MoveIt 轨迹。

现在还不能实现：

- 真机自动执行 MoveIt 轨迹。
- 稳定物理接触抓取。
- 自动闭合手指并根据接触反馈调整。
- 动态目标跟踪和视觉伺服。
- 失败后自动重规划。
- Isaac Sim 中完整识别-规划-接触抓取闭环。

## 8. 为什么不直接用原 grasp pipeline 执行

原 grasp pipeline 更像：

```text
目标点 -> g1-primitives IK -> 固定抓取动作
```

它能更快接近真机控制，但缺点是：

- 没有完整 MoveIt planning scene。
- 绕障能力弱。
- 不容易看完整关节轨迹。
- 不容易统一检查速度、加速度、碰撞、目标新鲜度。
- 和仿真/轨迹审查的连接不如 MoveIt 清晰。

现在新增 MoveIt 后变成：

```text
目标点 -> MoveIt 规划 -> 轨迹审查 -> RViz/Isaac Sim 验证 -> 再考虑真机执行
```

区别是：

- 更适合做工业化安全链路。
- 更容易接 Isaac Sim。
- 更容易把“识别”和“执行”解耦。
- 后续可以替换执行端，而不影响视觉端。

## 9. 真机执行前必须补的东西

在真机动之前，至少还需要：

1. Isaac Sim 中播放轨迹成功。
2. Isaac Sim 中加入桌子和目标物体。
3. Isaac Sim 中验证轨迹没有明显穿模。
4. 明确手指 open/close 时序。
5. 写真正的 Unitree trajectory execution bridge。
6. 执行桥必须 fail-closed。
7. 加 operator stop。
8. 加 DDS/control timeout。
9. 小幅度空载测试。
10. 最后再做真实抓取。

真机执行桥必须满足：

```text
规划失败 -> 不运动
轨迹审查失败 -> 不运动
目标过期 -> 不运动
目标和轨迹不一致 -> 不运动
碰撞状态无效 -> 不运动
关节映射缺失 -> 不运动
DDS 超时 -> 停止
人工停止 -> 停止
```

## 10. 当前最推荐的下一步

现在最合理的下一步不是接真机，而是：

```text
1. 修复网络，让 docker pull nvcr.io/nvidia/isaac-sim:5.1.0 成功。
2. 启动 Isaac Sim 容器。
3. 把 G1 URDF 导入为 USD。
4. 播放 last_plan_only_trajectory.json。
5. 在 Isaac Sim 里加桌子和目标物体。
6. 验证抓取动作和手部时序。
```

也就是：

```text
先仿真，再真机。
```

## 11. 一次完整实验记录模板

每次实验建议记录：

```text
日期：
目标物：
识别模式：file / ROS2
目标点：
MoveIt 规划是否成功：
轨迹点数：
轨迹时长：
review 是否通过：
pre_execution_gate 是否通过：
RViz 是否看到轨迹：
Isaac Sim 是否播放成功：
是否接触目标：
失败原因：
修改内容：
下一步：
```

当前一次已知成功的 MoveIt 轨迹示例：

```text
joint count: 10
trajectory points: 39
duration: 3.7212143s
target_xyz: [0.30, -0.14, 0.07]
waist_yaw_joint max_abs: 0.1149 rad
waist_roll_joint max_abs: 0.0869 rad
waist_pitch_joint max_abs: 0.0806 rad
file: /home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

当前一次已知成功的左手 MoveIt 轨迹示例：

```text
command source: /home/louisxx/g1_moveit_ws/runtime/left_test_target_xyz.txt
locked target: [0.35, 0.20, 0.18]
left pick target: [0.35, 0.22, 0.20]
group: left_arm
end_effector_link: left_hand_palm_link
joint count: 10
trajectory points: 23
duration: 2.199s
review: TRAJECTORY_REVIEW_PASSED
file: /home/louisxx/g1_moveit_ws/runtime/last_left_plan_only_trajectory.json
```

当前一次已知成功的 Isaac Sim GUI 播放记录：

```text
date: 2026-04-28
command: G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
result: Isaac Sim GUI loaded G1 USD and played the latest 39-point MoveIt trajectory.
terminal:
  Isaac Sim Full App is loaded.
  Playing trajectory: 39 points, joints=[...]
  Trajectory playback complete. Isaac Sim will stay open.
```

## 12. 文档索引

抓取/视觉工作区说明：

```text
/home/louisxx/g1_grasp_pipeline_workspace/README.md
```

MoveIt 单段详细流程：

```text
/home/louisxx/g1_moveit_ws/MOVEIT_WORKFLOW.md
```

Isaac Sim 操作说明：

```text
/home/louisxx/g1_moveit_ws/ISAAC_SIM_RUNBOOK.md
```

当前完整总流程：

```text
/home/louisxx/g1_moveit_ws/FULL_SYSTEM_WORKFLOW.md
```
