#!/usr/bin/env python3
"""Import the G1 MoveIt URDF into an Isaac Sim USD file.

Run inside the Isaac Sim container with:

  ./python.sh /workspace/g1_moveit_ws/isaacsim/import_g1_urdf.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


DEFAULT_URDF = Path("/workspace/g1_moveit_ws/src/g1_moveit_config/config/g1.urdf")
DEFAULT_OUTPUT = Path("/workspace/g1_moveit_ws/runtime/isaac/g1.usd")
DEFAULT_RESOLVED_URDF = Path("/workspace/g1_moveit_ws/runtime/isaac/g1_isaac_resolved.urdf")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output-usd", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--resolved-urdf", type=Path, default=DEFAULT_RESOLVED_URDF)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--fix-base", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def write_resolved_urdf(source: Path, dest: Path) -> None:
    text = source.read_text(encoding="utf-8")
    package_root = source.parent.parent
    text = text.replace("package://g1_moveit_config/", f"{package_root}/")
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(text, encoding="utf-8")


def set_if_exists(obj, name: str, value) -> None:
    if hasattr(obj, name):
        setattr(obj, name, value)


def main() -> None:
    args = parse_args()
    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)

    write_resolved_urdf(args.urdf, args.resolved_urdf)
    args.output_usd.parent.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

    import omni.kit.app
    import omni.kit.commands
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.stage import save_stage
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdLux, UsdPhysics

    enable_extension("isaacsim.asset.importer.urdf")
    for _ in range(10):
        app.update()

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")

    set_if_exists(import_config, "merge_fixed_joints", False)
    set_if_exists(import_config, "convex_decomp", False)
    set_if_exists(import_config, "import_inertia_tensor", True)
    set_if_exists(import_config, "fix_base", args.fix_base)
    set_if_exists(import_config, "distance_scale", 1.0)
    set_if_exists(import_config, "make_default_prim", True)
    set_if_exists(import_config, "self_collision", False)

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(args.resolved_urdf),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError(f"URDFParseAndImportFile failed for {args.resolved_urdf}")

    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physx_scene_api.CreateEnableCCDAttr(True)
    physx_scene_api.CreateEnableStabilizationAttr(True)
    physx_scene_api.CreateEnableGPUDynamicsAttr(False)
    physx_scene_api.CreateBroadphaseTypeAttr("MBP")
    physx_scene_api.CreateSolverTypeAttr("TGS")

    PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 2.0, Gf.Vec3f(0, 0, -0.78), Gf.Vec3f(0.5))
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    light.CreateIntensityAttr(500)

    try:
        from grasp_playback_common import apply_robot_visual_colors

        color_count = apply_robot_visual_colors(stage, "/g1")
        print(f"Applied robot visual colors to {color_count} prims")
    except Exception as exc:
        print(f"Could not apply robot visual colors: {exc}")

    for _ in range(5):
        app.update()

    save_stage(str(args.output_usd), save_and_reload_in_place=False)
    print(f"Imported G1 URDF: {args.resolved_urdf}")
    print(f"Articulation root: {prim_path}")
    print(f"Saved USD: {args.output_usd}")
    app.close()


if __name__ == "__main__":
    main()
