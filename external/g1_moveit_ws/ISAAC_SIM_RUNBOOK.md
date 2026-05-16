# G1 Isaac Sim Runbook

This document records the next validation step after the current MoveIt plan-only
pipeline: replay the saved MoveIt trajectory in Isaac Sim before any real robot
execution.

## 1. Host Preflight

Run on the host:

```bash
nvidia-smi
docker --version
docker run --rm --gpus all ubuntu nvidia-smi
```

Expected result:

- `nvidia-smi` shows the NVIDIA GPU and driver.
- Docker is installed.
- The test container can see the GPU.

If Docker cannot pull `ubuntu`, fix network or registry access first. Isaac Sim
images are large and also need registry access.

If Docker reports:

```text
failed to discover GPU vendor from CDI: no known GPU vendor found
```

then Docker is installed but the NVIDIA Container Toolkit is missing or not
configured. Install and configure it on the host:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Then verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi
```

## 2. Prepare a MoveIt Trajectory

Run on the host:

```bash
cd /home/louisxx/g1_moveit_ws
./run/03_demo_rviz.sh
```

In another terminal:

```bash
cd /home/louisxx/g1_moveit_ws
./run/04_static_scene.sh
./run/05_plan_only_target.sh
./run/06_review_last_trajectory.sh
./run/07_pre_execution_gate.sh
```

The playback input is:

```text
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

The default MoveIt trajectory is right-arm. To generate a left-arm trajectory,
run the same planning command with `G1_ARM=left`:

```bash
cd /home/louisxx/g1_moveit_ws
G1_ARM=left ./run/05_plan_only_target.sh
```

The standard Isaac GUI playback script reads:

```text
/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json
```

If you save a left-arm test trajectory to another file such as:

```text
/home/louisxx/g1_moveit_ws/runtime/last_left_plan_only_trajectory.json
```

copy or regenerate it into the standard path before using
`G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh`, or pass the custom trajectory to the
lower-level Isaac playback script.

The current recommended planner script enables waist constraints. This matters
because `right_arm` includes the waist joints. A plan can be mathematically
valid but visually wrong if the waist is allowed to twist freely.

Current known-good trajectory after adding waist constraints:

```text
target_xyz: [0.30, -0.14, 0.07]
point_count: 39
duration: 3.721s
waist_yaw_joint   max_abs=0.1149 rad
waist_roll_joint  max_abs=0.0869 rad
waist_pitch_joint max_abs=0.0806 rad
review: TRAJECTORY_REVIEW_PASSED
```

An older bad plan had:

```text
waist_yaw_joint   max_abs=1.9188 rad
waist_roll_joint  max_abs=0.5200 rad
waist_pitch_joint max_abs=0.5200 rad
```

That old plan should not be used for Isaac Sim review or real robot execution.

## 3. Pull Isaac Sim

Run on the host when network access is available:

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

If this fails with a TLS timeout, the current network cannot reach NVIDIA NGC
reliably. Fix the network/proxy and rerun the same command.

The launcher script defaults to that image. To use another version:

```bash
ISAACSIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.0.0 ./run/11_isaacsim_container_bash.sh
```

Current machine status:

```text
nvcr.io/nvidia/isaac-sim:5.1.0 has been pulled.
NVIDIA Container Toolkit has been installed/configured.
Full GUI launch works.
```

Latest validation:

```text
Date: 2026-04-28
Command: G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
Result: Isaac Sim GUI loaded G1 USD and played the latest 39-point MoveIt trajectory.
Terminal confirmation:
  Isaac Sim Full App is loaded.
  Playing trajectory: 39 points, joints=[...]
  Trajectory playback complete. Isaac Sim will stay open.
```

## 4. Start the Isaac Sim Container

Run on the host:

```bash
cd /home/louisxx/g1_moveit_ws
./run/11_isaacsim_container_bash.sh
```

Inside the container, the workspace is mounted at:

```text
/workspace/g1_moveit_ws
```

## 5. Import G1 URDF to USD

Recommended headless import from the host:

```bash
cd /home/louisxx/g1_moveit_ws
./run/14_isaacsim_import_g1_urdf.sh
```

This imports:

```text
/workspace/g1_moveit_ws/src/g1_moveit_config/config/g1.urdf
```

and saves:

```text
/workspace/g1_moveit_ws/runtime/isaac/g1.usd
```

Keep the joint names unchanged. The playback script matches joints by name, so
renaming joints during import will make the script stop with a mismatch report.

## 6. Replay the Last MoveIt Trajectory

Before entering Isaac Sim, check the host-side inputs:

```bash
cd /home/louisxx/g1_moveit_ws
./run/12_check_isaacsim_playback_inputs.sh
```

Inside the Isaac Sim container:

```bash
./python.sh /workspace/g1_moveit_ws/isaacsim/play_last_trajectory.py \
  --robot-usd /workspace/g1_moveit_ws/runtime/isaac/g1.usd
```

For headless validation:

```bash
./python.sh /workspace/g1_moveit_ws/isaacsim/play_last_trajectory.py \
  --robot-usd /workspace/g1_moveit_ws/runtime/isaac/g1.usd \
  --headless
```

Or run directly from the host:

```bash
cd /home/louisxx/g1_moveit_ws
./run/15_isaacsim_play_last_trajectory.sh --headless
```

For a GUI window on an X11 desktop:

```bash
xhost +local:docker
cd /home/louisxx/g1_moveit_ws
./run/17_isaacsim_gui_app.sh
```

The full GUI opens Isaac Sim and loads:

```text
/workspace/g1_moveit_ws/runtime/isaac/g1.usd
```

To open the full GUI and automatically replay the latest MoveIt trajectory:

```bash
xhost +local:docker
cd /home/louisxx/g1_moveit_ws
G1_ISAACSIM_EXEC=/workspace/g1_moveit_ws/isaacsim/open_and_play_g1_trajectory.py ./run/17_isaacsim_gui_app.sh
```

To run the adaptive grasp demo directly in GUI:

```bash
xhost +local:docker
cd /home/louisxx/g1_moveit_ws
./run/20_isaacsim_gui_adaptive_grasp.sh
```

Expected result:

- Isaac Sim loads the G1 USD.
- The waist and right-arm joints follow the saved MoveIt trajectory.
- With the latest trajectory, the waist should only move slightly. It should not
  do the previous large upper-body twist.
- The adaptive grasp demo spawns a small dynamic object, closes the fingers
  with feedback, and prints whether the object lifted successfully.
- If joint names do not match, the script prints missing MoveIt joints and the
  available Isaac articulation joints.

Useful terminal success lines:

```text
Simulation App Startup Complete
Playing trajectory: 39 points, joints=[...]
Trajectory playback complete. Isaac Sim will stay open.
```

Warnings about unresolved visual references such as `imu_in_pelvis`,
`imu_in_torso`, `d435_link`, or `mid360_link` can be ignored for this playback
stage. They are missing/sensor visual references and do not block joint
trajectory replay.

## 8. Jitter During Playback

If the hand or wrist jitters in Isaac Sim, distinguish two cases:

```text
large jump between trajectory points:
  usually caused by sparse point-by-point playback without frame interpolation

small high-frequency shaking:
  usually caused by Isaac articulation drive stiffness/damping, gravity,
  collision geometry, or uncontrolled finger joints
```

The current playback scripts now interpolate the MoveIt trajectory every frame
and keep writing the final joint positions after playback:

```text
isaacsim/open_and_play_g1_trajectory.py
isaacsim/play_last_trajectory.py
```

This makes the current Isaac step a visualization playback, not a full physics
controller. If shaking remains, the next fix is to tune the imported USD
articulation drives and explicitly lock or control the hand/finger joints.

The current GUI playback also holds all non-trajectory joints at their initial
positions. Only the MoveIt trajectory joints are allowed to move:

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

This prevents the left arm, legs, and unused hand joints from drifting during
trajectory visualization. If the left hand still moves after this change, the
remaining cause is in the imported USD joint drives/collision settings rather
than the MoveIt trajectory.

## 7. What This Validates

This validates only:

- MoveIt trajectory can be loaded outside ROS.
- Isaac Sim robot joint names match the MoveIt trajectory.
- The saved path can be replayed in a physics scene.

It does not yet validate:

- Real Unitree DDS command publishing.
- Hand close/open timing.
- Contact-stable object grasping.
- Camera perception inside Isaac Sim.

Those should be added after this minimal playback succeeds.
