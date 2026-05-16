from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


VALID_EVENTS = {"open", "close", "release"}
VALID_HANDS = {"left", "right"}


def load_trajectory(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid trajectory JSON: {path}")
    return data


def validate_hand_events(data: dict, *, require_grasp_sequence: bool) -> tuple[list[str], list[dict]]:
    errors: list[str] = []
    summary = data.get("summary", {})
    if not isinstance(summary, dict):
        errors.append("trajectory has no summary dictionary")
        summary = {}

    sequence_type = summary.get("sequence_type")
    if require_grasp_sequence and sequence_type != "grasp_plan_only":
        errors.append(f"summary.sequence_type must be 'grasp_plan_only', got {sequence_type!r}")

    duration = float(summary.get("duration", 0.0))
    raw_events = data.get("hand_events", [])
    if not isinstance(raw_events, list) or not raw_events:
        errors.append("trajectory has no hand_events list")
        return errors, []

    events: list[dict] = []
    previous_time = -1.0
    for index, event in enumerate(raw_events):
        if not isinstance(event, dict):
            errors.append(f"hand_events[{index}] is not a dictionary")
            continue
        event_name = event.get("event")
        hand = event.get("hand")
        try:
            event_time = float(event.get("time_from_start"))
        except (TypeError, ValueError):
            errors.append(f"hand_events[{index}] has invalid time_from_start: {event.get('time_from_start')!r}")
            continue

        if event_name not in VALID_EVENTS:
            errors.append(f"hand_events[{index}] has unsupported event {event_name!r}")
        if hand not in VALID_HANDS:
            errors.append(f"hand_events[{index}] has unsupported hand {hand!r}")
        if event_time < 0.0:
            errors.append(f"hand_events[{index}] has negative time {event_time:.6f}")
        if event_time < previous_time:
            errors.append(
                f"hand_events[{index}] time {event_time:.6f} is before previous event time {previous_time:.6f}"
            )
        if duration > 0.0 and event_time > duration:
            errors.append(f"hand_events[{index}] time {event_time:.6f} exceeds trajectory duration {duration:.6f}")

        previous_time = event_time
        events.append({"event": event_name, "hand": hand, "time_from_start": event_time})

    return errors, events


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dry-run hand event bridge. Prints open/close/release timing only.")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--hand-mode", choices=("inspire", "dex3"), default="inspire")
    parser.add_argument("--require-grasp-sequence", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--report", default="/home/louisxx/g1_moveit_ws/runtime/hand_events_dry_run_report.json")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    data = load_trajectory(args.trajectory)
    summary = data.get("summary", {})
    errors, events = validate_hand_events(data, require_grasp_sequence=args.require_grasp_sequence)

    report = {
        "trajectory": str(Path(args.trajectory).resolve()),
        "sequence_type": summary.get("sequence_type") if isinstance(summary, dict) else None,
        "duration": summary.get("duration") if isinstance(summary, dict) else None,
        "hand_mode": args.hand_mode,
        "events": events,
        "errors": errors,
        "valid": not errors,
    }
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("DRY_RUN_HAND_EVENTS_BRIDGE")
    print("No DDS initialization. No hand command will be sent.")
    print(f"trajectory: {Path(args.trajectory).resolve()}")
    print(f"sequence_type: {report['sequence_type']}")
    print(f"duration: {float(report['duration'] or 0.0):.3f}s")
    print(f"hand_mode: {args.hand_mode}")
    print("events:")
    for index, event in enumerate(events):
        print(f"  {index:02d}: t={event['time_from_start']:.3f}s hand={event['hand']} event={event['event']}")
    print(f"report: {Path(args.report).resolve()}")

    if errors:
        print("DRY_RUN_HAND_EVENTS_BLOCKED")
        for error in errors:
            print(f"  - {error}")
        sys.exit(2)

    print("DRY_RUN_HAND_EVENTS_READY")


if __name__ == "__main__":
    main()
