from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


DEFAULT_TRAJECTORY = Path("/workspace/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
DEFAULT_ROBOT_PRIM = "/g1"
DEFAULT_OBJECT_PRIM = "/World/AdaptiveGraspObject"
HAND_TRANSITION_SEC = 0.8

ROBOT_VISUAL_COLORS = {
    "pelvis": (0.14, 0.14, 0.16),
    "torso": (0.16, 0.16, 0.18),
    "hip": (0.16, 0.16, 0.18),
    "thigh": (0.20, 0.20, 0.22),
    "calf": (0.18, 0.18, 0.20),
    "ankle": (0.16, 0.16, 0.18),
    "foot": (0.12, 0.12, 0.13),
    "arm": (0.62, 0.64, 0.67),
    "shoulder": (0.62, 0.64, 0.67),
    "elbow": (0.56, 0.58, 0.61),
    "hand": (0.28, 0.28, 0.30),
    "finger": (0.33, 0.33, 0.36),
    "sensor": (0.08, 0.08, 0.09),
    "camera": (0.08, 0.08, 0.09),
    "lidar": (0.08, 0.08, 0.09),
    "imu": (0.08, 0.08, 0.09),
}
LEFT_HAND_JOINTS = [
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
]
RIGHT_HAND_JOINTS = [
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
]
LEFT_OPEN = [0.0] * 7
RIGHT_OPEN = [0.0] * 7
LEFT_CLOSED = [0.60, 0.60, 1.20, -1.20, -1.40, -1.20, -1.40]
RIGHT_CLOSED = [-0.60, -0.60, -1.20, 1.20, 1.40, 1.20, 1.40]


def load_trajectory(path: Path) -> tuple[dict, list[str], list[dict], list[dict]]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    summary = data.get("summary", {})
    joint_names = data.get("joint_names") or summary.get("joint_names")
    points = data.get("points")
    hand_events = data.get("hand_events", [])
    if not joint_names or not points:
        raise RuntimeError(f"Invalid trajectory file: {path}")
    return summary if isinstance(summary, dict) else {}, list(joint_names), list(points), list(hand_events)


def point_time(point: dict) -> float:
    return float(point.get("time_from_start", 0.0))


def interpolate_positions(points: list[dict], elapsed: float, first_t: float, playback_speed: float) -> list[float]:
    trajectory_t = first_t + elapsed * playback_speed
    if trajectory_t <= point_time(points[0]):
        return [float(value) for value in points[0]["positions"]]
    if trajectory_t >= point_time(points[-1]):
        return [float(value) for value in points[-1]["positions"]]

    for next_i in range(1, len(points)):
        previous = points[next_i - 1]
        current = points[next_i]
        t0 = point_time(previous)
        t1 = point_time(current)
        if trajectory_t <= t1:
            alpha = 0.0 if t1 <= t0 else (trajectory_t - t0) / (t1 - t0)
            q0 = [float(value) for value in previous["positions"]]
            q1 = [float(value) for value in current["positions"]]
            return [(1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1)]
    return [float(value) for value in points[-1]["positions"]]


def lerp(a: list[float], b: list[float], alpha: float) -> list[float]:
    alpha = max(0.0, min(1.0, alpha))
    return [(1.0 - alpha) * float(x) + alpha * float(y) for x, y in zip(a, b)]


def hand_target(hand: str, event: str) -> list[float]:
    if hand == "left":
        return LEFT_OPEN if event in {"open", "release"} else LEFT_CLOSED
    return RIGHT_OPEN if event in {"open", "release"} else RIGHT_CLOSED


def event_events_by_hand(events: list[dict]) -> dict[str, list[dict]]:
    grouped = {"left": [], "right": []}
    for event in sorted(events, key=lambda item: float(item.get("time_from_start", 0.0))):
        hand = event.get("hand")
        if hand in grouped:
            grouped[hand].append(event)
    return grouped


def read_joint_vector(robot, method_names: list[str]) -> np.ndarray | None:
    for name in method_names:
        if hasattr(robot, name):
            try:
                values = getattr(robot, name)()
                return np.array(values, dtype=float)
            except Exception:
                continue
    return None


def read_robot_state(robot) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    positions = read_joint_vector(robot, ["get_joint_positions"])
    velocities = read_joint_vector(robot, ["get_joint_velocities"])
    efforts = read_joint_vector(robot, ["get_measured_joint_efforts", "get_joint_efforts"])
    return positions, velocities, efforts


class JointTargetCommander:
    def __init__(self, robot, *, prefer_direct_positions: bool = False) -> None:
        self.robot = robot
        self.controller = None
        self.articulation_action = None
        self.prefer_direct_positions = prefer_direct_positions
        try:
            from isaacsim.core.utils.types import ArticulationAction

            self.articulation_action = ArticulationAction
        except Exception:
            try:
                from omni.isaac.core.utils.types import ArticulationAction  # type: ignore

                self.articulation_action = ArticulationAction
            except Exception:
                self.articulation_action = None
        if hasattr(robot, "get_articulation_controller"):
            try:
                self.controller = robot.get_articulation_controller()
            except Exception:
                self.controller = None

    def apply(self, joint_positions: list[float]) -> None:
        if not self.prefer_direct_positions and self.controller is not None and self.articulation_action is not None:
            try:
                self.controller.apply_action(self.articulation_action(joint_positions=joint_positions))
                return
            except Exception:
                pass

        self.robot.set_joint_positions(joint_positions)
        try:
            joint_velocities = self.robot.get_joint_velocities()
            joint_velocities[:] = 0.0
            self.robot.set_joint_velocities(joint_velocities)
        except Exception:
            pass


@dataclass
class AdaptiveGraspConfig:
    close_transition_sec: float = 1.2
    release_transition_sec: float = 0.8
    contact_effort_threshold: float = 0.85
    contact_error_threshold: float = 0.025
    stall_velocity_threshold: float = 0.02
    lock_squeeze: float = 0.01
    enable_contact_lock: bool = True
    enable_lift_check: bool = True
    lift_check_delay_sec: float = 0.7
    lift_success_z: float = 0.015


class AdaptiveHandState:
    def __init__(self, hand: str, config: AdaptiveGraspConfig) -> None:
        self.hand = hand
        self.config = config
        self.contact_locked = False
        self.lock_positions: list[float] | None = None
        self.contact_time: float | None = None
        self.close_event_time: float | None = None
        self.release_event_time: float | None = None

    def update_schedule(self, events: list[dict]) -> None:
        close_time = None
        release_time = None
        for event in sorted(events, key=lambda item: float(item.get("time_from_start", 0.0))):
            if event.get("hand") != self.hand:
                continue
            event_name = event.get("event")
            event_time = float(event.get("time_from_start", 0.0))
            if event_name == "close":
                close_time = event_time
                release_time = None
            elif event_name in {"release", "open"} and close_time is not None:
                release_time = event_time
                break
        self.close_event_time = close_time
        self.release_event_time = release_time

    def is_close_phase(self, trajectory_t: float) -> bool:
        if self.close_event_time is None:
            return False
        if trajectory_t < self.close_event_time:
            return False
        if self.release_event_time is not None and trajectory_t >= self.release_event_time:
            return False
        return True

    def hand_command(
        self,
        trajectory_t: float,
        measured_positions: np.ndarray | None,
        measured_velocities: np.ndarray | None,
        measured_efforts: np.ndarray | None,
    ) -> list[float]:
        if self.hand == "left":
            open_positions = LEFT_OPEN
            closed_positions = LEFT_CLOSED
        else:
            open_positions = RIGHT_OPEN
            closed_positions = RIGHT_CLOSED

        if not self.is_close_phase(trajectory_t):
            self.contact_locked = False
            self.lock_positions = None
            self.contact_time = None
            if self.release_event_time is None:
                return list(open_positions)
            if trajectory_t < self.release_event_time + self.config.release_transition_sec:
                progress = (trajectory_t - self.release_event_time) / self.config.release_transition_sec
                return lerp(closed_positions, open_positions, progress)
            return list(open_positions)

        if self.contact_locked and self.lock_positions is not None:
            if self.config.enable_lift_check and self.contact_time is not None and trajectory_t > self.contact_time + self.config.lift_check_delay_sec:
                return list(self.lock_positions)
            return list(self.lock_positions)

        progress = 0.0
        if self.close_event_time is not None:
            progress = (trajectory_t - self.close_event_time) / self.config.close_transition_sec
        progress = max(0.0, min(1.0, progress))
        command = lerp(open_positions, closed_positions, progress)

        if self.config.enable_contact_lock and measured_positions is not None:
            if self._contact_detected(command, measured_positions, measured_velocities, measured_efforts, progress):
                self.contact_locked = True
                self.contact_time = trajectory_t
                self.lock_positions = measured_positions.tolist()
                return list(self.lock_positions)

        return command

    def _contact_detected(
        self,
        command_positions: list[float],
        measured_positions: np.ndarray,
        measured_velocities: np.ndarray | None,
        measured_efforts: np.ndarray | None,
        progress: float,
    ) -> bool:
        if progress < 0.25:
            return False

        target = np.array(command_positions, dtype=float)
        error = np.max(np.abs(target - measured_positions))
        if error <= self.config.contact_error_threshold:
            return True

        if measured_efforts is not None and measured_efforts.size:
            effort_peak = float(np.max(np.abs(measured_efforts)))
            if effort_peak >= self.config.contact_effort_threshold:
                return True

        if measured_velocities is not None and measured_velocities.size:
            velocity_peak = float(np.max(np.abs(measured_velocities)))
            if velocity_peak <= self.config.stall_velocity_threshold and error > self.config.contact_error_threshold:
                return True

        return False


def spawn_grasp_object(stage, *, center_xyz: list[float] | tuple[float, float, float], prim_path: str = DEFAULT_OBJECT_PRIM, size: float = 0.04):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        return prim_path

    cube = UsdGeom.Cube.Define(stage, Sdf.Path(prim_path))
    cube.CreateSizeAttr(float(size))
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3f(float(center_xyz[0]), float(center_xyz[1]), float(center_xyz[2])))

    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    if hasattr(rigid_api, "CreateDisableGravityAttr"):
        rigid_api.CreateDisableGravityAttr(True)
    if hasattr(rigid_api, "CreateKinematicEnabledAttr"):
        rigid_api.CreateKinematicEnabledAttr(True)
    if hasattr(rigid_api, "CreateLinearDampingAttr"):
        rigid_api.CreateLinearDampingAttr(0.15)
    if hasattr(rigid_api, "CreateAngularDampingAttr"):
        rigid_api.CreateAngularDampingAttr(0.15)

    return prim_path


def read_prim_translation(stage, prim_path: str) -> np.ndarray | None:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    cache = UsdGeom.XformCache()
    transform = cache.GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]], dtype=float)


def set_prim_translation(stage, prim_path: str, xyz: list[float] | tuple[float, float, float]) -> bool:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return False
    try:
        return bool(UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))))
    except Exception:
        return False


def add_checker_floor(
    stage,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
    z: float = -0.78,
    half_extent: float = 4.0,
    tile_size: float = 0.5,
    prim_root: str = "/World/CheckerFloor",
) -> int:
    from pxr import Gf, Sdf, UsdGeom

    if half_extent <= 0.0 or tile_size <= 0.0:
        return 0

    tile_count = int(math.ceil((half_extent * 2.0) / tile_size))
    start_x = float(center_xy[0] - half_extent)
    start_y = float(center_xy[1] - half_extent)
    applied = 0

    for ix in range(tile_count):
        for iy in range(tile_count):
            x0 = start_x + ix * tile_size
            y0 = start_y + iy * tile_size
            x1 = min(x0 + tile_size, center_xy[0] + half_extent)
            y1 = min(y0 + tile_size, center_xy[1] + half_extent)
            if x1 <= x0 or y1 <= y0:
                continue

            prim_path = f"{prim_root}/tile_{ix}_{iy}"
            mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(prim_path))
            points = [
                Gf.Vec3f(x0, y0, z),
                Gf.Vec3f(x1, y0, z),
                Gf.Vec3f(x1, y1, z),
                Gf.Vec3f(x0, y1, z),
            ]
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr([4])
            mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateSubdivisionSchemeAttr("none")
            rgb = (0.36, 0.36, 0.38) if (ix + iy) % 2 == 0 else (0.28, 0.28, 0.30)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
            if hasattr(mesh, "CreateDisplayOpacityAttr"):
                mesh.CreateDisplayOpacityAttr([1.0])
            applied += 1

    return applied


def apply_robot_visual_colors(stage, robot_root: str = "/g1") -> int:
    from pxr import Gf, UsdGeom

    root = stage.GetPrimAtPath(robot_root)
    if not root.IsValid():
        return 0

    applied = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Gprim):
            continue
        path_text = prim.GetPath().pathString.lower()
        if not path_text.startswith(robot_root.lower()):
            continue

        rgb = None
        for key, color in ROBOT_VISUAL_COLORS.items():
            if key in path_text:
                rgb = color
                break
        if rgb is None:
            if "hand" in path_text or "finger" in path_text:
                rgb = (0.24, 0.24, 0.26)
            elif "arm" in path_text or "shoulder" in path_text or "elbow" in path_text:
                rgb = (0.60, 0.62, 0.65)
            elif "leg" in path_text or "hip" in path_text or "knee" in path_text or "ankle" in path_text:
                rgb = (0.18, 0.18, 0.20)
            else:
                rgb = (0.58, 0.58, 0.60)

        gprim = UsdGeom.Gprim(prim)
        gprim.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        if hasattr(gprim, "CreateDisplayOpacityAttr"):
            gprim.CreateDisplayOpacityAttr([1.0])
        applied += 1

    return applied
