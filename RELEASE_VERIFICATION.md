# Release Verification

Verified before upload:

- `run/00_check_prereqs.sh` passed.
- `run/20_smoke_fake_target_to_plan.sh` passed and generated a MoveIt trajectory from a fake ROS2 target.

Runtime logs, Python bytecode caches, virtual environments, build folders, and Git metadata are intentionally excluded from this repository copy.

The related `g1_moveit_ws` source dependency is packaged under `external/g1_moveit_ws/` without generated `build/`, `install/`, `log/`, or `runtime/` outputs. The verified local smoke test used the already-built MoveIt overlay at `MOVEIT_WS=/home/louisxx/g1_moveit_ws`.
