"""Open G1 USD in the full Isaac Sim GUI and replay the latest MoveIt trajectory."""

from __future__ import annotations

import argparse
import asyncio
import time
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import omni.kit.app
import omni.usd

from grasp_playback_common import (
    AdaptiveGraspConfig,
    AdaptiveHandState,
    add_checker_floor,
    apply_robot_visual_colors,
    DEFAULT_OBJECT_PRIM,
    DEFAULT_ROBOT_PRIM,
    DEFAULT_TRAJECTORY,
    JointTargetCommander,
    interpolate_positions,
    load_trajectory,
    point_time,
    read_prim_translation,
    read_robot_state,
    spawn_grasp_object,
)


USD_PATH = "/workspace/g1_moveit_ws/runtime/isaac/g1.usd"
ROBOT_PRIM_PATH = DEFAULT_ROBOT_PRIM
TRAJECTORY_PATH = DEFAULT_TRAJECTORY


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, default=TRAJECTORY_PATH)
    parser.add_argument("--robot-usd", type=Path, default=Path(USD_PATH))
    parser.add_argument("--robot-prim-path", default=ROBOT_PRIM_PATH)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 120.0)
    parser.add_argument("--playback-speed", type=float, default=0.35)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--keep-open", action="store_true")
    parser.add_argument("--adaptive-grasp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--spawn-grasp-object", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--grasp-object-prim-path", default=DEFAULT_OBJECT_PRIM)
    parser.add_argument("--grasp-object-size", type=float, default=0.04)
    parser.add_argument("--contact-effort-threshold", type=float, default=0.85)
    parser.add_argument("--contact-error-threshold", type=float, default=0.025)
    parser.add_argument("--stall-velocity-threshold", type=float, default=0.02)
    parser.add_argument("--lift-success-z", type=float, default=0.015)
    args, _ = parser.parse_known_args()
    return args


def build_full_positions(hold_positions, source_indices, source_positions, hand_indices, hand_positions):
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


async def main() -> None:
    args = parse_args()
    app = omni.kit.app.get_app()
    await app.next_update_async()

    print(f"Opening G1 stage: {USD_PATH}", flush=True)
    omni.usd.get_context().open_stage(str(args.robot_usd))
    stage = omni.usd.get_context().get_stage()
    for _ in range(180):
        await app.next_update_async()
        if stage is not None and stage.GetPrimAtPath(DEFAULT_ROBOT_PRIM).IsValid():
            break
    if stage is None or not stage.GetPrimAtPath(DEFAULT_ROBOT_PRIM).IsValid():
        raise RuntimeError(f"Robot prim not found at {DEFAULT_ROBOT_PRIM} after opening stage")

    try:
        tile_count = add_checker_floor(stage, center_xy=(0.0, 0.0), z=-0.78, half_extent=4.0, tile_size=0.5)
        print(f"Checker floor added: {tile_count} tiles", flush=True)
    except Exception as exc:
        print(f"Could not add checker floor: {exc}", flush=True)

    try:
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(45.0)
        dome.CreateColorAttr(Gf.Vec3f(0.18, 0.18, 0.20))
        print("Dome light added", flush=True)
    except Exception as exc:
        print(f"Could not add dome light: {exc}", flush=True)

    try:
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
        key.CreateIntensityAttr(560.0)
        key.CreateColorAttr(Gf.Vec3f(0.96, 0.95, 0.92))
        key_xform = UsdGeom.Xformable(key.GetPrim())
        key_xform.AddRotateXYZOp().Set(Gf.Vec3f(-42.0, 0.0, 28.0))
        print("Key light added", flush=True)
    except Exception as exc:
        print(f"Could not add key light: {exc}", flush=True)

    try:
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        fill = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/FillLight"))
        fill.CreateIntensityAttr(180.0)
        fill.CreateColorAttr(Gf.Vec3f(0.88, 0.90, 0.95))
        fill_xform = UsdGeom.Xformable(fill.GetPrim())
        fill_xform.AddRotateXYZOp().Set(Gf.Vec3f(25.0, 0.0, -110.0))
        print("Fill light added", flush=True)
    except Exception as exc:
        print(f"Could not add fill light: {exc}", flush=True)

    try:
        color_count = apply_robot_visual_colors(stage, DEFAULT_ROBOT_PRIM)
        print(f"Applied robot visual colors to {color_count} prims", flush=True)
    except Exception as exc:
        print(f"Could not apply robot visual colors: {exc}", flush=True)

    try:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=[2.0, -2.6, 1.2], target=[0.0, 0.0, 0.25])
    except Exception as exc:
        print(f"Could not set viewport camera: {exc}", flush=True)

    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.prims import SingleArticulation
    import omni.timeline as timeline_mod

    summary, joint_names, points, hand_events = load_trajectory(args.trajectory)
    timeline = timeline_mod.get_timeline_interface()
    timeline.play()
    print("Timeline playing", flush=True)
    SimulationManager.initialize_physics()
    print("Physics initialization requested", flush=True)
    for _ in range(20):
        await app.next_update_async()
    robot = SingleArticulation(prim_path=args.robot_prim_path, name="g1")
    robot.initialize()
    print("Robot articulation initialized", flush=True)
    for _ in range(30):
        await app.next_update_async()

    articulation_joint_names = list(robot.dof_names)
    missing = [name for name in joint_names if name not in articulation_joint_names]
    if missing:
        print("MoveIt joints missing from Isaac articulation:", flush=True)
        for name in missing:
            print(f"  - {name}", flush=True)
        print("Isaac articulation joints:", flush=True)
        for name in articulation_joint_names:
            print(f"  - {name}", flush=True)
        return

    indices = [articulation_joint_names.index(name) for name in joint_names]
    hand_indices = {
        "left": [articulation_joint_names.index(name) for name in ("left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint", "left_hand_middle_0_joint", "left_hand_middle_1_joint", "left_hand_index_0_joint", "left_hand_index_1_joint") if name in articulation_joint_names],
        "right": [articulation_joint_names.index(name) for name in ("right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint", "right_hand_middle_0_joint", "right_hand_middle_1_joint", "right_hand_index_0_joint", "right_hand_index_1_joint") if name in articulation_joint_names],
    }
    if len(hand_indices["left"]) != 7 or len(hand_indices["right"]) != 7:
        print("Some hand joints are missing from Isaac articulation; finger closure may be incomplete.", flush=True)

    hold_positions = robot.get_joint_positions().copy()
    hold_velocities = robot.get_joint_velocities()
    hold_velocities[:] = 0.0
    robot.set_joint_velocities(hold_velocities)
    commander = JointTargetCommander(robot, prefer_direct_positions=True)

    adaptive_cfg = AdaptiveGraspConfig(
        contact_effort_threshold=args.contact_effort_threshold,
        contact_error_threshold=args.contact_error_threshold,
        stall_velocity_threshold=args.stall_velocity_threshold,
        lift_success_z=args.lift_success_z,
    )
    hand_states = {
        "left": AdaptiveHandState("left", adaptive_cfg),
        "right": AdaptiveHandState("right", adaptive_cfg),
    }
    for state in hand_states.values():
        state.update_schedule(hand_events)

    grasp_object_prim = None
    if args.adaptive_grasp and args.spawn_grasp_object:
        center_xyz = summary.get("pick_xyz") or summary.get("target_xyz")
        if isinstance(center_xyz, list) and len(center_xyz) == 3:
            grasp_object_prim = spawn_grasp_object(
                stage,
                center_xyz=center_xyz,
                prim_path=args.grasp_object_prim_path,
                size=args.grasp_object_size,
            )
            object_start_xyz = read_prim_translation(stage, grasp_object_prim)
        else:
            object_start_xyz = None
    else:
        object_start_xyz = None

    print(f"Playing trajectory: {len(points)} points, joints={joint_names}", flush=True)
    print(f"Hand events: {hand_events}", flush=True)
    print(f"Adaptive grasp: {args.adaptive_grasp}", flush=True)
    if grasp_object_prim:
        print(f"Grasp object prim: {grasp_object_prim}", flush=True)

    start_wall = time.monotonic()
    first_t = point_time(points[0])
    playback_speed = max(args.playback_speed, 1e-6)
    duration = max(0.0, point_time(points[-1]) - first_t) / playback_speed
    last_measured = read_robot_state(robot)
    while time.monotonic() - start_wall <= duration:
        elapsed = time.monotonic() - start_wall
        trajectory_t = first_t + elapsed * playback_speed
        source_positions = interpolate_positions(points, elapsed, first_t, playback_speed)
        measured_positions, measured_velocities, measured_efforts = last_measured
        hand_positions = {}
        for hand, state in hand_states.items():
            indices_for_hand = hand_indices[hand]
            current = measured_positions[indices_for_hand] if measured_positions is not None and indices_for_hand else None
            current_vel = measured_velocities[indices_for_hand] if measured_velocities is not None and indices_for_hand else None
            current_eff = measured_efforts[indices_for_hand] if measured_efforts is not None and indices_for_hand else None
            hand_positions[hand] = state.hand_command(trajectory_t, current, current_vel, current_eff)

        full_positions = build_full_positions(hold_positions, indices, source_positions, hand_indices, hand_positions)
        commander.apply(full_positions.tolist() if hasattr(full_positions, "tolist") else list(full_positions))
        await app.next_update_async()
        last_measured = read_robot_state(robot)

    final_positions = [float(value) for value in points[-1]["positions"]]
    final_t = point_time(points[-1])
    final_hand_positions = {}
    measured_positions, measured_velocities, measured_efforts = last_measured
    for hand, state in hand_states.items():
        indices_for_hand = hand_indices[hand]
        current = measured_positions[indices_for_hand] if measured_positions is not None and indices_for_hand else None
        current_vel = measured_velocities[indices_for_hand] if measured_velocities is not None and indices_for_hand else None
        current_eff = measured_efforts[indices_for_hand] if measured_efforts is not None and indices_for_hand else None
        final_hand_positions[hand] = state.hand_command(final_t, current, current_vel, current_eff)

    hold_until = time.monotonic() + args.hold_seconds
    while time.monotonic() < hold_until and app.is_running():
        full_positions = build_full_positions(hold_positions, indices, final_positions, hand_indices, final_hand_positions)
        commander.apply(full_positions.tolist() if hasattr(full_positions, "tolist") else list(full_positions))
        await app.next_update_async()

    if grasp_object_prim and object_start_xyz is not None:
        object_final_xyz = read_prim_translation(stage, grasp_object_prim)
        if object_final_xyz is not None:
            delta_z = float(object_final_xyz[2] - object_start_xyz[2])
            if delta_z >= args.lift_success_z:
                print(f"ADAPTIVE_GRASP_SUCCESS object_lift_z={delta_z:.4f}m", flush=True)
            else:
                print(f"ADAPTIVE_GRASP_RETRY_REQUIRED object_lift_z={delta_z:.4f}m", flush=True)

    while args.keep_open and app.is_running():
        full_positions = build_full_positions(hold_positions, indices, final_positions, hand_indices, final_hand_positions)
        commander.apply(full_positions.tolist() if hasattr(full_positions, "tolist") else list(full_positions))
        await app.next_update_async()


asyncio.ensure_future(main())
