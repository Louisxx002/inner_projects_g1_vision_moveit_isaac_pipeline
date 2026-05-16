#!/usr/bin/env python3
"""Check host-side inputs before running Isaac Sim playback."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    workspace_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=workspace_root / "runtime" / "last_plan_only_trajectory.json",
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=workspace_root / "runtime" / "isaac" / "g1.usd",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    ok = True
    if not args.trajectory.exists():
        print(f"ERROR: missing trajectory: {args.trajectory}")
        ok = False
    else:
        with args.trajectory.open("r", encoding="utf-8") as f:
            data = json.load(f)
        joint_names = data.get("joint_names") or data.get("summary", {}).get("joint_names") or []
        points = data.get("points") or []
        print(f"trajectory: {args.trajectory}")
        print(f"  joints: {len(joint_names)}")
        print(f"  points: {len(points)}")
        if points:
            print(f"  duration: {points[-1].get('time_from_start', 0.0)}s")
        if not joint_names or not points:
            print("ERROR: trajectory must contain joint_names and points")
            ok = False

    if not args.robot_usd.exists():
        print(f"WARN: robot USD does not exist yet: {args.robot_usd}")
        print("      Import the G1 URDF in Isaac Sim and save it to this path before playback.")
    else:
        print(f"robot USD: {args.robot_usd}")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
