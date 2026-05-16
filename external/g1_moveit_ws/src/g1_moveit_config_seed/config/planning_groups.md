# G1 MoveIt Planning Groups Draft

Use these names in MoveIt Setup Assistant.

## Base Link

```text
pelvis
```

## Right Arm

Group name:

```text
right_arm
```

Kinematic chain:

```text
base link: pelvis
tip link : right_hand_palm_link
```

Joints:

```text
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
```

## Left Arm

Group name:

```text
left_arm
```

Kinematic chain:

```text
base link: pelvis
tip link : left_hand_palm_link
```

Joints:

```text
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_joint
left_wrist_roll_joint
left_wrist_pitch_joint
left_wrist_yaw_joint
```

## Dual Arm

Group name:

```text
dual_arm
```

Joints:

```text
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_joint
left_wrist_roll_joint
left_wrist_pitch_joint
left_wrist_yaw_joint
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_joint
right_wrist_roll_joint
right_wrist_pitch_joint
right_wrist_yaw_joint
```

Create `right_arm` first and validate it in RViz before adding dual-arm
planning.
