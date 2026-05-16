"""Open the generated G1 USD after Isaac Sim GUI startup."""

from __future__ import annotations

import asyncio

import omni.kit.app
import omni.usd


USD_PATH = "/workspace/g1_moveit_ws/runtime/isaac/g1.usd"


async def main() -> None:
    app = omni.kit.app.get_app()
    for _ in range(30):
        await app.next_update_async()

    print(f"Opening G1 stage: {USD_PATH}", flush=True)
    omni.usd.get_context().open_stage(USD_PATH)

    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        try:
            from grasp_playback_common import add_checker_floor, apply_robot_visual_colors
            from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdLux

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

        try:
            color_count = apply_robot_visual_colors(stage, "/g1")
            print(f"Applied robot visual colors to {color_count} prims", flush=True)
        except Exception as exc:
            print(f"Could not apply robot visual colors: {exc}", flush=True)

    for _ in range(90):
        await app.next_update_async()

    try:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=[3.0, -3.0, 2.0], target=[0.0, 0.0, 0.75])
    except Exception as exc:
        print(f"Could not set viewport camera: {exc}", flush=True)


asyncio.ensure_future(main())
