#!/usr/bin/env python3
"""Minimal Isaac Sim startup test for the G1 workspace."""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--renderer", default="RaytracedLighting")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print(
        f"SMOKE_TEST_START headless={args.headless} renderer={args.renderer}",
        flush=True,
    )

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "renderer": args.renderer})
    try:
        print("SMOKE_TEST_SIMULATION_APP_READY", flush=True)
        app.update()
        print("SMOKE_TEST_UPDATE_OK", flush=True)
    finally:
        app.close()

    print("SMOKE_TEST_PASSED", flush=True)


if __name__ == "__main__":
    main()
