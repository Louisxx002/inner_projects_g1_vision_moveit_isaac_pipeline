# Safety Boundary

This workspace is allowed to do:

```text
Read locked target files.
Validate workspace limits.
Publish MoveIt planning-scene collision objects.
Send MoveIt plan-only requests.
Inspect returned trajectories.
```

This workspace is not allowed to do yet:

```text
Publish Unitree DDS commands.
Publish rt/arm_sdk.
Publish rt/lowcmd.
Control Inspire/Dex3 hands.
Execute MoveIt trajectories.
Bridge FollowJointTrajectory to robot hardware.
```

Before any execution bridge is added, implement and review:

```text
1. Joint-name order mapping from MoveIt to Unitree arm_sdk.
2. Joint position bounds check for every trajectory point.
3. Velocity and acceleration bounds check.
4. Trajectory timestamp monotonicity check.
5. Planning-scene collision check immediately before execution.
6. Stale target timeout.
7. Operator stop input.
8. DDS timeout stop behavior.
9. Dry-run log replay mode.
10. Low-speed first-motion mode.
```

Execution policy for any future bridge:

```text
Use control_mode=arm-sdk only.
Publish upper-body arm joints through rt/arm_sdk only.
Do not publish rt/lowcmd from this workspace.
Keep the official lower-body controller running.
```

The first accepted milestone is:

```text
RViz can plan right_arm or left_arm to a locked target while avoiding static collision boxes.
No real robot motion happens.
```
