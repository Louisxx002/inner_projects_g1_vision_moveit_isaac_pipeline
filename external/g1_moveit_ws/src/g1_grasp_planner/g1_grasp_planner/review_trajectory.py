from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml


def load_joint_limits(path: str | Path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("joint_limits"), dict):
        raise ValueError(f"Invalid joint limits file: {path}")
    return data["joint_limits"]


def require_finite_vector(name: str, values: list[float], expected_size: int) -> None:
    if len(values) != expected_size:
        raise ValueError(f"{name} expected {expected_size} values, got {len(values)}")
    bad = [value for value in values if not math.isfinite(float(value))]
    if bad:
        raise ValueError(f"{name} contains non-finite values: {bad}")


def review_trajectory(args: argparse.Namespace) -> tuple[list[str], dict]:
    data = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    summary = data.get("summary", {})
    joints = summary.get("joint_names", [])
    points = data.get("points", [])
    limits = load_joint_limits(args.joint_limits)

    errors: list[str] = []
    warnings: list[str] = []

    if not joints:
        errors.append("trajectory has no joint_names")
    if len(set(joints)) != len(joints):
        errors.append("trajectory joint_names contain duplicates")
    if len(points) < 2:
        errors.append(f"trajectory has too few points: {len(points)}")

    missing_limits = [joint for joint in joints if joint not in limits]
    if missing_limits:
        errors.append(f"missing joint_limits entries: {missing_limits}")

    previous_time = None
    previous_positions = None
    max_declared_velocity = 0.0
    max_declared_acceleration = 0.0
    max_segment_velocity = 0.0
    min_dt = None
    joint_positions: dict[str, list[float]] = {joint: [] for joint in joints}

    for point_index, point in enumerate(points):
        t = float(point.get("time_from_start", float("nan")))
        positions = [float(value) for value in point.get("positions", [])]
        velocities = [float(value) for value in point.get("velocities", [])]
        accelerations = [float(value) for value in point.get("accelerations", [])]

        try:
            require_finite_vector(f"point[{point_index}].positions", positions, len(joints))
            require_finite_vector(f"point[{point_index}].velocities", velocities, len(joints))
            require_finite_vector(f"point[{point_index}].accelerations", accelerations, len(joints))
        except ValueError as exc:
            errors.append(str(exc))
            continue

        if not math.isfinite(t):
            errors.append(f"point[{point_index}] has non-finite time_from_start: {t}")
            continue

        if previous_time is None:
            if abs(t) > args.time_epsilon:
                warnings.append(f"first point time is {t:.6f}s, expected 0.0s")
        else:
            dt = t - previous_time
            if dt <= args.time_epsilon:
                errors.append(f"time is not strictly increasing at point[{point_index}]: dt={dt:.9f}")
            else:
                min_dt = dt if min_dt is None else min(min_dt, dt)
                if previous_positions is not None:
                    for joint, q0, q1 in zip(joints, previous_positions, positions):
                        segment_velocity = abs(q1 - q0) / dt
                        max_segment_velocity = max(max_segment_velocity, segment_velocity)
                        limit = limits.get(joint, {})
                        if limit.get("has_velocity_limits"):
                            allowed = float(limit.get("max_velocity", 0.0)) * args.velocity_scale * args.tolerance
                            if segment_velocity > allowed:
                                errors.append(
                                    f"{joint} segment velocity {segment_velocity:.6f} exceeds scaled limit {allowed:.6f} "
                                    f"between point[{point_index - 1}] and point[{point_index}]"
                                )

        for joint_index, joint in enumerate(joints):
            joint_positions.setdefault(joint, []).append(positions[joint_index])
            limit = limits.get(joint, {})
            velocity = abs(velocities[joint_index])
            acceleration = abs(accelerations[joint_index])
            max_declared_velocity = max(max_declared_velocity, velocity)
            max_declared_acceleration = max(max_declared_acceleration, acceleration)

            if limit.get("has_velocity_limits"):
                allowed_velocity = float(limit.get("max_velocity", 0.0)) * args.velocity_scale * args.tolerance
                if velocity > allowed_velocity:
                    errors.append(
                        f"{joint} declared velocity {velocity:.6f} exceeds scaled limit {allowed_velocity:.6f} "
                        f"at point[{point_index}]"
                    )
            else:
                warnings.append(f"{joint} has no velocity limit in joint_limits.yaml")

            if limit.get("has_acceleration_limits"):
                allowed_acceleration = float(limit.get("max_acceleration", 0.0)) * args.acceleration_scale * args.tolerance
                if acceleration > allowed_acceleration:
                    errors.append(
                        f"{joint} declared acceleration {acceleration:.6f} exceeds scaled limit {allowed_acceleration:.6f} "
                        f"at point[{point_index}]"
                    )
            else:
                warnings.append(f"{joint} has no acceleration limit in joint_limits.yaml")

        previous_time = t
        previous_positions = positions

    joint_stats = {}
    for joint, values in joint_positions.items():
        if not values:
            continue
        start = values[0]
        end = values[-1]
        min_value = min(values)
        max_value = max(values)
        delta = end - start
        max_abs = max(abs(value) for value in values)
        max_abs_delta_from_start = max(abs(value - start) for value in values)
        joint_stats[joint] = {
            "start": start,
            "end": end,
            "min": min_value,
            "max": max_value,
            "delta": delta,
            "max_abs": max_abs,
            "max_abs_delta_from_start": max_abs_delta_from_start,
        }

    waist_limits = {
        "waist_yaw_joint": args.max_waist_yaw_abs,
        "waist_roll_joint": args.max_waist_roll_abs,
        "waist_pitch_joint": args.max_waist_pitch_abs,
    }
    for joint, max_abs_allowed in waist_limits.items():
        stats = joint_stats.get(joint)
        if stats is None:
            continue
        if stats["max_abs"] > max_abs_allowed:
            errors.append(
                f"{joint} max_abs {stats['max_abs']:.6f} exceeds natural waist limit {max_abs_allowed:.6f}"
            )
        if stats["max_abs_delta_from_start"] > args.max_waist_delta:
            errors.append(
                f"{joint} moved {stats['max_abs_delta_from_start']:.6f}rad from start, "
                f"exceeds natural waist delta {args.max_waist_delta:.6f}"
            )

    report = {
        "trajectory": str(Path(args.trajectory).resolve()),
        "joint_limits": str(Path(args.joint_limits).resolve()),
        "joint_count": len(joints),
        "point_count": len(points),
        "duration": float(points[-1]["time_from_start"]) if points else 0.0,
        "min_dt": min_dt,
        "max_declared_velocity": max_declared_velocity,
        "max_declared_acceleration": max_declared_acceleration,
        "max_segment_velocity": max_segment_velocity,
        "joint_stats": joint_stats,
        "warnings": sorted(set(warnings)),
        "errors": errors,
    }
    return errors, report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Review a saved MoveIt plan-only trajectory JSON before any execution bridge.")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--joint-limits", default="/home/louisxx/g1_moveit_ws/src/g1_moveit_config/config/joint_limits.yaml")
    parser.add_argument("--velocity-scale", type=float, default=0.15)
    parser.add_argument("--acceleration-scale", type=float, default=0.15)
    parser.add_argument("--tolerance", type=float, default=1.05)
    parser.add_argument("--time-epsilon", type=float, default=1e-9)
    parser.add_argument("--max-waist-yaw-abs", type=float, default=0.45)
    parser.add_argument("--max-waist-roll-abs", type=float, default=0.25)
    parser.add_argument("--max-waist-pitch-abs", type=float, default=0.25)
    parser.add_argument("--max-waist-delta", type=float, default=0.35)
    parser.add_argument("--report", default="/home/louisxx/g1_moveit_ws/runtime/last_trajectory_review.json")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    errors, report = review_trajectory(args)
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"trajectory: {report['trajectory']}")
    print(f"points: {report['point_count']} duration: {report['duration']:.3f}s min_dt: {report['min_dt']}")
    print(f"max_declared_velocity: {report['max_declared_velocity']:.6f} rad/s")
    print(f"max_declared_acceleration: {report['max_declared_acceleration']:.6f} rad/s^2")
    print(f"max_segment_velocity: {report['max_segment_velocity']:.6f} rad/s")
    if report["joint_stats"]:
        print("joint_stats:")
        for joint, stats in report["joint_stats"].items():
            print(
                f"  - {joint}: start={stats['start']:.4f} end={stats['end']:.4f} "
                f"min={stats['min']:.4f} max={stats['max']:.4f} "
                f"delta={stats['delta']:.4f} max_abs={stats['max_abs']:.4f}"
            )
    print(f"report: {Path(args.report).resolve()}")

    if report["warnings"]:
        print("warnings:")
        for warning in report["warnings"]:
            print(f"  - {warning}")

    if errors:
        print("TRAJECTORY_REVIEW_FAILED")
        for error in errors:
            print(f"  - {error}")
        sys.exit(2)

    print("TRAJECTORY_REVIEW_PASSED")


if __name__ == "__main__":
    main()
