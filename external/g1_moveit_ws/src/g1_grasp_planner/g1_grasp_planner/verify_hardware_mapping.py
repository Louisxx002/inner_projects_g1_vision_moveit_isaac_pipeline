from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]


def load_yaml(path: str | Path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML file: {path}")
    return data


def load_trajectory_joint_names(path: str | Path) -> list[str]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    joint_names = data.get("summary", {}).get("joint_names")
    if not isinstance(joint_names, list) or not joint_names:
        raise ValueError(f"Trajectory has no summary.joint_names: {path}")
    return [str(name) for name in joint_names]


def verify_mapping(joint_names: list[str], mapping: dict) -> tuple[list[str], list[dict]]:
    errors: list[str] = []
    mapped_rows: list[dict] = []

    control_mode = mapping.get("control_mode")
    command_topic = mapping.get("command_topic")
    if control_mode != "arm-sdk":
        errors.append(f"control_mode must be 'arm-sdk' to preserve the official lower-body controller, got {control_mode!r}")
    if command_topic != "rt/arm_sdk":
        errors.append(f"command_topic must be 'rt/arm_sdk' in arm-sdk mode, got {command_topic!r}")
    if command_topic == "rt/lowcmd":
        errors.append("rt/lowcmd is not allowed in this MoveIt workspace because it can take over the lower body")

    moveit_to_unitree = mapping.get("moveit_to_unitree")
    if not isinstance(moveit_to_unitree, dict):
        return ["mapping file has no moveit_to_unitree dictionary"], []

    seen_indices: dict[int, str] = {}
    for order, joint_name in enumerate(joint_names):
        item = moveit_to_unitree.get(joint_name)
        if not isinstance(item, dict):
            errors.append(f"missing mapping for trajectory joint: {joint_name}")
            continue
        unitree_index = item.get("unitree_index")
        unitree_name = item.get("unitree_name")
        if not isinstance(unitree_index, int):
            errors.append(f"{joint_name} has invalid unitree_index: {unitree_index!r}")
            continue
        if unitree_index < 0 or unitree_index > 34:
            errors.append(f"{joint_name} unitree_index outside G1_29 range 0..34: {unitree_index}")
        if unitree_index in seen_indices:
            errors.append(f"duplicate unitree_index {unitree_index}: {seen_indices[unitree_index]} and {joint_name}")
        seen_indices[unitree_index] = joint_name
        mapped_rows.append(
            {
                "trajectory_order": order,
                "moveit_joint": joint_name,
                "unitree_name": unitree_name,
                "unitree_index": unitree_index,
            }
        )

    weight_joint = mapping.get("weight_joint", {})
    weight_index = weight_joint.get("unitree_index")
    if not isinstance(weight_index, int):
        errors.append("weight_joint.unitree_index is missing or invalid")
    elif weight_index in seen_indices:
        errors.append(f"weight joint index {weight_index} conflicts with trajectory joint {seen_indices[weight_index]}")

    return errors, mapped_rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Verify MoveIt trajectory joint names against Unitree G1 motor indices.")
    parser.add_argument("--trajectory", default=str(WORKSPACE_ROOT / "runtime" / "last_plan_only_trajectory.json"))
    parser.add_argument("--mapping", default=str(WORKSPACE_ROOT / "config" / "unitree_g1_29_joint_map.yaml"))
    parser.add_argument("--report", default=str(WORKSPACE_ROOT / "runtime" / "hardware_mapping_report.json"))
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mapping = load_yaml(args.mapping)
    joint_names = load_trajectory_joint_names(args.trajectory)
    errors, rows = verify_mapping(joint_names, mapping)
    report = {
        "trajectory": str(Path(args.trajectory).resolve()),
        "mapping": str(Path(args.mapping).resolve()),
        "robot": mapping.get("robot"),
        "control_mode": mapping.get("control_mode"),
        "command_topic": mapping.get("command_topic"),
        "state_topic": mapping.get("state_topic"),
        "lower_body_controller": mapping.get("lower_body_controller"),
        "weight_joint": mapping.get("weight_joint"),
        "mapped_joints": rows,
        "errors": errors,
        "valid": not errors,
    }
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"mapping_report: {Path(args.report).resolve()}")
    print(f"robot: {report['robot']}")
    print(f"control_mode: {report['control_mode']}")
    print(f"command_topic: {report['command_topic']}")
    print(f"state_topic: {report['state_topic']}")
    print(f"lower_body_controller: {report['lower_body_controller']}")
    print("trajectory_order -> unitree motor index:")
    for row in rows:
        print(
            f"  {row['trajectory_order']:02d}: "
            f"{row['moveit_joint']} -> {row['unitree_name']}[{row['unitree_index']}]"
        )

    if errors:
        print("HARDWARE_MAPPING_FAILED")
        for error in errors:
            print(f"  - {error}")
        sys.exit(2)

    print("HARDWARE_MAPPING_PASSED")


if __name__ == "__main__":
    main()
