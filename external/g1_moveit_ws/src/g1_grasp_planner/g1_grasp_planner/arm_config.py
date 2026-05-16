from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmConfig:
    group_name: str
    end_effector_link: str
    pick_offset: tuple[float, float, float]


ARM_CONFIGS = {
    "right": ArmConfig(
        group_name="right_arm",
        end_effector_link="right_hand_palm_link",
        pick_offset=(0.0, -0.08, 0.04),
    ),
    "left": ArmConfig(
        group_name="left_arm",
        end_effector_link="left_hand_palm_link",
        pick_offset=(0.0, 0.08, 0.04),
    ),
}


DEFAULT_TARGET_HAND_FILE = "/home/louisxx/g1_grasp_pipeline_workspace/runtime/locked_target_hand.txt"
DEFAULT_HAND_DEADBAND_M = 0.02
ARM_CHOICES = ("auto", *tuple(sorted(ARM_CONFIGS)))


def resolve_arm_config(arm: str) -> ArmConfig:
    try:
        return ARM_CONFIGS[arm]
    except KeyError as exc:
        choices = ", ".join(sorted(ARM_CONFIGS))
        raise ValueError(f"Unsupported arm={arm!r}; choose one of: {choices}") from exc


def infer_arm_from_target_y(target_xyz, deadband_m: float = DEFAULT_HAND_DEADBAND_M) -> str:
    y = float(target_xyz[1])
    if y > deadband_m:
        return "left"
    if y < -deadband_m:
        return "right"
    raise ValueError(
        f"Locked target y={y:.4f} is within +/-{deadband_m:.4f} m of center. "
        "Use --arm left or --arm right for this target."
    )


def read_target_hand_file(path: str) -> str | None:
    from pathlib import Path

    hand_path = Path(path)
    if not hand_path.exists():
        return None
    text = hand_path.read_text(encoding="utf-8").strip()
    if not text:
        return None
    hand = text.split()[0].lower()
    if hand in ARM_CONFIGS:
        return hand
    if hand == "center":
        raise ValueError(
            f"Target hand file {hand_path} says center. "
            "Use --arm left or --arm right for this target."
        )
    raise ValueError(f"Target hand file {hand_path} must contain left, right, or center, got {hand!r}")


def resolve_requested_arm(requested_arm: str, target_xyz, target_hand_file: str, deadband_m: float) -> str:
    if requested_arm != "auto":
        return requested_arm

    file_arm = read_target_hand_file(target_hand_file)
    if file_arm is not None:
        return file_arm
    return infer_arm_from_target_y(target_xyz, deadband_m)
