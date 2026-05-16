#!/usr/bin/env python3
"""Play the latest MoveIt plan-only trajectory inside Isaac Sim."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from grasp_playback_common import (
    AdaptiveGraspConfig,
    AdaptiveHandState,
    add_checker_floor,
    apply_robot_visual_colors,
    DEFAULT_ROBOT_PRIM,
    DEFAULT_TRAJECTORY,
    JointTargetCommander,
    interpolate_positions,
    load_trajectory,
    point_time,
    read_robot_state,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY)
    parser.add_argument("--robot-usd", type=Path, required=True)
    parser.add_argument("--robot-prim-path", default=DEFAULT_ROBOT_PRIM)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 120.0)
    parser.add_argument("--playback-speed", type=float, default=0.35)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--keep-open", action="store_true")
    parser.add_argument("--gui-kinematic", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--adaptive-grasp", action=argparse.BooleanOptionalAction, default=True)
    args, _ = parser.parse_known_args()
    return args


def build_full_positions(
    hold_positions,
    source_indices: list[int],
    source_positions: list[float],
    hand_indices: dict[str, list[int]],
    hand_positions: dict[str, list[float]],
):
    full_positions = hold_positions.copy()
    for source_i, dof_i in enumerate(source_indices):
        full_positions[dof_i] = float(source_positions[source_i])
    for hand, dof_indices in hand_indices.items():
        values = hand_positions.get(hand)
        if values is None:
            continue
        for value, dof_i in zip(values, dof_indices):
            full_positions[dof_i] = float(value)
    return full_positions


def add_demo_environment(stage) -> None:
    from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdLux

    try:
        PhysicsSchemaTools.addGroundPlane(
            stage,
            "/World/GroundPlane",
            "Z",
            2.0,
            Gf.Vec3f(0.0, 0.0, -0.78),
            Gf.Vec3f(0.5),
        )
        print("Ground plane added", flush=True)
    except Exception as exc:
        print(f"Could not add ground plane: {exc}", flush=True)

    try:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(45.0)
        dome.CreateColorAttr(Gf.Vec3f(0.18, 0.18, 0.20))
        print("Dome light added", flush=True)
    except Exception as exc:
        print(f"Could not add dome light: {exc}", flush=True)

    try:
        key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
        key.CreateIntensityAttr(560.0)
        key.CreateColorAttr(Gf.Vec3f(0.96, 0.95, 0.92))
        key_xform = UsdGeom.Xformable(key.GetPrim())
        key_xform.AddRotateXYZOp().Set(Gf.Vec3f(-42.0, 0.0, 28.0))
        print("Key light added", flush=True)
    except Exception as exc:
        print(f"Could not add key light: {exc}", flush=True)

    try:
        fill = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/FillLight"))
        fill.CreateIntensityAttr(180.0)
        fill.CreateColorAttr(Gf.Vec3f(0.88, 0.90, 0.95))
        fill_xform = UsdGeom.Xformable(fill.GetPrim())
        fill_xform.AddRotateXYZOp().Set(Gf.Vec3f(25.0, 0.0, -110.0))
        print("Fill light added", flush=True)
    except Exception as exc:
        print(f"Could not add fill light: {exc}", flush=True)

    try:
        tile_count = add_checker_floor(stage, center_xy=(0.0, 0.0), z=-0.78, half_extent=4.0, tile_size=0.5)
        print(f"Checker floor added: {tile_count} tiles", flush=True)
    except Exception as exc:
        print(f"Could not add checker floor: {exc}", flush=True)


def main() -> None:
    args = parse_args()
    if not args.trajectory.exists():
        raise FileNotFoundError(args.trajectory)
    if not args.robot_usd.exists():
        raise FileNotFoundError(args.robot_usd)

    summary, joint_names, points, hand_events = load_trajectory(args.trajectory)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    print("Simulation App Starting", flush=True)
    for i in range(5):
        print(f"Simulation warmup step {i + 1}/5", flush=True)
        simulation_app.update()
    print("Simulation warmup done", flush=True)

    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.viewports import set_camera_view
    import omni.usd

    print(f"Adding robot USD: {args.robot_usd} -> {args.robot_prim_path}", flush=True)
    add_reference_to_stage(usd_path=str(args.robot_usd), prim_path=args.robot_prim_path)
    print("Robot USD reference added", flush=True)
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        add_demo_environment(stage)
        color_count = apply_robot_visual_colors(stage, args.robot_prim_path)
        print(f"Applied robot visual colors to {color_count} prims", flush=True)
    for i in range(120):
        if i % 10 == 0:
            print(f"Waiting for robot prim... step {i + 1}/120", flush=True)
        simulation_app.update()
        if stage is not None and stage.GetPrimAtPath(args.robot_prim_path).IsValid():
            break
    print("Robot prim wait loop finished", flush=True)
    if stage is None or not stage.GetPrimAtPath(args.robot_prim_path).IsValid():
        raise RuntimeError(f"Robot prim not found at {args.robot_prim_path}")

    robot = SingleArticulation(prim_path=args.robot_prim_path, name="g1")
    world = None
    if args.gui_kinematic or args.headless:
        from isaacsim.core.api import World

        print(f"Creating World on prim {args.robot_prim_path}", flush=True)
        world = World(stage_units_in_meters=1.0, physics_dt=args.physics_dt, rendering_dt=args.physics_dt)
        world.scene.add(robot)
        world.reset()
    robot.initialize()
    if args.gui_kinematic and world is not None:
        print("GUI kinematic world enabled", flush=True)
    for _ in range(30):
        simulation_app.update()
    if not args.headless:
        try:
            from isaacsim.core.utils.viewports import set_camera_view

            set_camera_view(eye=[3.0, -3.0, 2.0], target=[0.0, 0.0, 0.75])
            print("Viewport camera set", flush=True)
        except Exception as exc:
            print(f"Could not set viewport camera: {exc}", flush=True)

    articulation_joint_names = list(robot.dof_names)
    missing = [name for name in joint_names if name not in articulation_joint_names]
    if missing:
        print("MoveIt joints missing from Isaac articulation:")
        for name in missing:
            print(f"  - {name}")
        print("Isaac articulation joints:")
        for name in articulation_joint_names:
            print(f"  - {name}")
        raise RuntimeError("joint name mismatch; fix the URDF import/USD joint names before playback")

    indices = [articulation_joint_names.index(name) for name in joint_names]
    hand_indices = {
        "left": [articulation_joint_names.index(name) for name in ("left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint", "left_hand_middle_0_joint", "left_hand_middle_1_joint", "left_hand_index_0_joint", "left_hand_index_1_joint") if name in articulation_joint_names],
        "right": [articulation_joint_names.index(name) for name in ("right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint", "right_hand_middle_0_joint", "right_hand_middle_1_joint", "right_hand_index_0_joint", "right_hand_index_1_joint") if name in articulation_joint_names],
    }
    if len(hand_indices["left"]) != 7 or len(hand_indices["right"]) != 7:
        print("Some hand joints are missing from Isaac articulation; finger closure may be incomplete.")

    hold_positions = robot.get_joint_positions().copy()
    hold_velocities = robot.get_joint_velocities()
    hold_velocities[:] = 0.0
    robot.set_joint_velocities(hold_velocities)
    commander = JointTargetCommander(robot, prefer_direct_positions=args.gui_kinematic)

    adaptive_cfg = AdaptiveGraspConfig()
    if args.gui_kinematic:
        adaptive_cfg.enable_contact_lock = False
        adaptive_cfg.enable_lift_check = False
    hand_states = {
        "left": AdaptiveHandState("left", adaptive_cfg),
        "right": AdaptiveHandState("right", adaptive_cfg),
    }
    for state in hand_states.values():
        state.update_schedule(hand_events)

    print(f"Loaded trajectory: {len(points)} points, joints={joint_names}", flush=True)
    print(f"Hand events: {hand_events}", flush=True)
    print(f"Adaptive grasp: {args.adaptive_grasp}", flush=True)
    print(f"Robot USD: {args.robot_usd}", flush=True)
    print(f"Playback speed scale: {args.playback_speed}", flush=True)
    print("Playback loop starting", flush=True)

    for _ in range(30):
        simulation_app.update()

    start_wall = time.monotonic()
    first_t = point_time(points[0])
    playback_speed = max(args.playback_speed, 1e-6)
    duration = max(0.0, point_time(points[-1]) - first_t) / playback_speed
    last_measured = read_robot_state(robot)
    last_command_positions = None
    last_command_wall = start_wall
    next_progress_log = start_wall + 5.0
    while time.monotonic() - start_wall <= duration and simulation_app.is_running():
        now = time.monotonic()
        elapsed = now - start_wall
        trajectory_t = first_t + elapsed * playback_speed
        source_positions = interpolate_positions(points, elapsed, first_t, playback_speed)
        measured_positions, measured_velocities, measured_efforts = last_measured
        hand_positions = {}
        for hand, state in hand_states.items():
            if hand == "left":
                indices_for_hand = hand_indices["left"]
            else:
                indices_for_hand = hand_indices["right"]
            if not indices_for_hand:
                hand_positions[hand] = []
                continue
            current = None
            current_vel = None
            current_eff = None
            if measured_positions is not None:
                current = measured_positions[indices_for_hand]
            if measured_velocities is not None:
                current_vel = measured_velocities[indices_for_hand]
            if measured_efforts is not None:
                current_eff = measured_efforts[indices_for_hand]
            hand_positions[hand] = state.hand_command(trajectory_t, current, current_vel, current_eff)

        target_positions = build_full_positions(hold_positions, indices, source_positions, hand_indices, hand_positions)
        target_positions = target_positions.tolist() if hasattr(target_positions, "tolist") else list(target_positions)
        if args.gui_kinematic:
            dt = max(0.0, min(now - last_command_wall, 0.12))
            alpha = 1.0 - math.exp(-dt / 0.10) if dt > 0.0 else 1.0
            if last_command_positions is None:
                command_positions = target_positions
            else:
                command_positions = [
                    prev + alpha * (target - prev)
                    for prev, target in zip(last_command_positions, target_positions)
                ]
            if last_command_positions is not None:
                delta = max(abs(target - prev) for target, prev in zip(command_positions, last_command_positions))
                if delta < 1e-4:
                    command_positions = last_command_positions
            last_command_positions = list(command_positions)
            last_command_wall = now
        else:
            command_positions = target_positions
        commander.apply(command_positions)
        simulation_app.update()
        last_measured = read_robot_state(robot)
        if time.monotonic() >= next_progress_log:
            print(
                f"Playback progress elapsed={elapsed:.2f}s trajectory_t={trajectory_t:.2f}s",
                flush=True,
            )
            next_progress_log = time.monotonic() + 5.0

    final_positions = [float(value) for value in points[-1]["positions"]]
    final_hand_positions = {}
    final_t = point_time(points[-1])
    for hand, state in hand_states.items():
        if hand == "left":
            indices_for_hand = hand_indices["left"]
        else:
            indices_for_hand = hand_indices["right"]
        current = None
        current_vel = None
        current_eff = None
        if last_measured[0] is not None:
            current = last_measured[0][indices_for_hand]
        if last_measured[1] is not None:
            current_vel = last_measured[1][indices_for_hand]
        if last_measured[2] is not None:
            current_eff = last_measured[2][indices_for_hand]
        final_hand_positions[hand] = state.hand_command(final_t, current, current_vel, current_eff)

    hold_until = time.monotonic() + args.hold_seconds
    while time.monotonic() < hold_until and simulation_app.is_running():
        full_positions = build_full_positions(hold_positions, indices, final_positions, hand_indices, final_hand_positions)
        command_positions = full_positions.tolist() if hasattr(full_positions, "tolist") else list(full_positions)
        if args.gui_kinematic:
            if last_command_positions is None:
                last_command_positions = list(command_positions)
            else:
                command_positions = [
                    prev + 0.15 * (target - prev)
                    for prev, target in zip(last_command_positions, command_positions)
                ]
                last_command_positions = list(command_positions)
        commander.apply(command_positions)
        simulation_app.update()

    print("Playback loop complete", flush=True)

    while args.keep_open and simulation_app.is_running():
        full_positions = build_full_positions(hold_positions, indices, final_positions, hand_indices, final_hand_positions)
        command_positions = full_positions.tolist() if hasattr(full_positions, "tolist") else list(full_positions)
        if args.gui_kinematic:
            if last_command_positions is None:
                last_command_positions = list(command_positions)
            else:
                command_positions = [
                    prev + 0.15 * (target - prev)
                    for prev, target in zip(last_command_positions, command_positions)
                ]
                last_command_positions = list(command_positions)
        commander.apply(command_positions)
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
