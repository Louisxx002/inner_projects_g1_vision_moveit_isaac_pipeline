from __future__ import annotations

import argparse
import json
import sys
import time
from multiprocessing import Array
from pathlib import Path

import numpy as np


GRASP_WORKSPACE = Path("/home/louisxx/g1_grasp_pipeline_workspace")
if str(GRASP_WORKSPACE) not in sys.path:
    sys.path.insert(0, str(GRASP_WORKSPACE))

from examples.basic_usage import (  # noqa: E402
    DEX3_LEFT_CLOSED_POS,
    DEX3_LEFT_OPEN_POS,
    DEX3_RIGHT_CLOSED_POS,
    DEX3_RIGHT_OPEN_POS,
    INSPIRE_LEFT_CLOSED_POS,
    INSPIRE_LEFT_OPEN_POS,
    INSPIRE_RIGHT_CLOSED_POS,
    INSPIRE_RIGHT_OPEN_POS,
    NETWORK_INTERFACE,
    DOMAIN_ID,
    SimpleDex3Controller,
    SimpleInspireController,
)


VALID_EVENTS = {"open", "close", "release"}
VALID_HANDS = {"left", "right"}


def load_events(path: str | Path) -> tuple[dict, list[dict]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    summary = data.get("summary", {})
    events = data.get("hand_events", [])
    if not isinstance(summary, dict):
        raise ValueError("trajectory summary is missing or invalid")
    if summary.get("sequence_type") != "grasp_plan_only":
        raise ValueError(f"expected summary.sequence_type='grasp_plan_only', got {summary.get('sequence_type')!r}")
    if not isinstance(events, list) or not events:
        raise ValueError("trajectory has no hand_events")
    normalized = []
    previous_time = -1.0
    duration = float(summary.get("duration", 0.0))
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            raise ValueError(f"hand_events[{index}] is not a dictionary")
        event_name = event.get("event")
        hand = event.get("hand")
        event_time = float(event.get("time_from_start"))
        if event_name not in VALID_EVENTS:
            raise ValueError(f"hand_events[{index}] has unsupported event {event_name!r}")
        if hand not in VALID_HANDS:
            raise ValueError(f"hand_events[{index}] has unsupported hand {hand!r}")
        if event_time < previous_time:
            raise ValueError(f"hand_events[{index}] time is not monotonic")
        if event_time < 0.0 or (duration > 0.0 and event_time > duration):
            raise ValueError(f"hand_events[{index}] time {event_time:.3f}s outside trajectory duration {duration:.3f}s")
        previous_time = event_time
        normalized.append({"event": event_name, "hand": hand, "time_from_start": event_time})
    return summary, normalized


def defaults_for_mode(hand_mode: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if hand_mode == "inspire":
        return (
            INSPIRE_LEFT_OPEN_POS.copy(),
            INSPIRE_RIGHT_OPEN_POS.copy(),
            INSPIRE_LEFT_CLOSED_POS.copy(),
            INSPIRE_RIGHT_CLOSED_POS.copy(),
        )
    if hand_mode == "dex3":
        return (
            DEX3_LEFT_OPEN_POS.copy(),
            DEX3_RIGHT_OPEN_POS.copy(),
            DEX3_LEFT_CLOSED_POS.copy(),
            DEX3_RIGHT_CLOSED_POS.copy(),
        )
    raise ValueError(f"unsupported hand_mode: {hand_mode}")


def set_array(array: Array, values: np.ndarray) -> None:
    with array.get_lock():
        array[:] = values.astype(float).tolist()


def get_array(array: Array) -> np.ndarray:
    with array.get_lock():
        return np.array(array[:], dtype=float)


def interpolate_array(array: Array, target: np.ndarray, duration: float, rate_hz: float) -> None:
    start = get_array(array)
    steps = max(1, int(duration * rate_hz))
    period = 1.0 / rate_hz
    for values in np.linspace(start, target, steps):
        set_array(array, values)
        time.sleep(period)
    set_array(array, target)


def event_target(event_name: str, hand: str, left_open: np.ndarray, right_open: np.ndarray, left_closed: np.ndarray, right_closed: np.ndarray) -> np.ndarray:
    if event_name in {"open", "release"}:
        return left_open if hand == "left" else right_open
    if event_name == "close":
        return left_closed if hand == "left" else right_closed
    raise ValueError(f"unsupported event: {event_name}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Execute or dry-run hand_events from a MoveIt grasp sequence.")
    parser.add_argument("--trajectory", default="/home/louisxx/g1_moveit_ws/runtime/last_plan_only_trajectory.json")
    parser.add_argument("--hand-mode", choices=("inspire", "dex3"), default="inspire")
    parser.add_argument("--enable-live-hand", action="store_true", help="Actually initialize DDS and command the hand.")
    parser.add_argument("--respect-timing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--time-scale", type=float, default=1.0, help="Scale event times. Use 0 for immediate event playback.")
    parser.add_argument("--transition-duration", type=float, default=0.8)
    parser.add_argument("--publish-rate", type=float, default=100.0)
    parser.add_argument("--event-filter", choices=("all", "open", "close", "release"), default="all")
    parser.add_argument("--hold-after", type=float, default=1.0)
    parser.add_argument("--network-interface", default=NETWORK_INTERFACE)
    parser.add_argument("--domain-id", type=int, default=DOMAIN_ID)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    summary, events = load_events(args.trajectory)
    if args.event_filter != "all":
        events = [event for event in events if event["event"] == args.event_filter]

    print("HAND_EVENTS_LIVE_BRIDGE")
    print(f"trajectory: {Path(args.trajectory).resolve()}")
    print(f"sequence_type: {summary.get('sequence_type')}")
    print(f"hand_mode: {args.hand_mode}")
    print(f"enable_live_hand: {args.enable_live_hand}")
    print(f"network_interface: {args.network_interface}")
    print(f"domain_id: {args.domain_id}")
    print("events:")
    for index, event in enumerate(events):
        print(f"  {index:02d}: t={event['time_from_start']:.3f}s hand={event['hand']} event={event['event']}")

    if not args.enable_live_hand:
        print("LIVE_HAND_DISABLED")
        print("No DDS initialization. Re-run with --enable-live-hand, or ENABLE_LIVE_HAND=1 via run script, to command the hand.")
        return

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(args.domain_id, networkInterface=args.network_interface)
    left_open, right_open, left_closed, right_closed = defaults_for_mode(args.hand_mode)
    dof = len(left_open)
    left_array = Array("d", dof)
    right_array = Array("d", dof)
    set_array(left_array, left_open)
    set_array(right_array, right_open)

    controller = (
        SimpleInspireController(left_array, right_array, fps=args.publish_rate)
        if args.hand_mode == "inspire"
        else SimpleDex3Controller(left_array, right_array, fps=args.publish_rate)
    )
    start_time = time.monotonic()
    try:
        for event in events:
            scheduled_time = float(event["time_from_start"]) * max(0.0, args.time_scale)
            if args.respect_timing:
                sleep_time = scheduled_time - (time.monotonic() - start_time)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
            target = event_target(event["event"], event["hand"], left_open, right_open, left_closed, right_closed)
            array = left_array if event["hand"] == "left" else right_array
            print(
                f"EXEC_HAND_EVENT t={time.monotonic() - start_time:.3f}s "
                f"hand={event['hand']} event={event['event']} target={target.tolist()}",
                flush=True,
            )
            interpolate_array(array, target, args.transition_duration, args.publish_rate)
        if args.hold_after > 0.0:
            time.sleep(args.hold_after)
    finally:
        controller.close()
    print("LIVE_HAND_EVENTS_COMPLETE")


if __name__ == "__main__":
    main()
