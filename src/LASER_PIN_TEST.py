#!/usr/bin/env python3
"""
Minimal Orange Pi laser pin test using the WiringOP gpio command.

Default laser pin in this repo:
- laser: wPi 10

Examples:
  python3 LASER_PIN_TEST.py on
  python3 LASER_PIN_TEST.py off
  python3 LASER_PIN_TEST.py pulse
  python3 LASER_PIN_TEST.py blink --times 5 --interval 0.3
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time


LASER_PIN = 10


def run_gpio(args: list[str], gpio_command: str) -> None:
    if shutil.which(gpio_command) is None:
        raise RuntimeError(
            f"GPIO command '{gpio_command}' not found. Install the Orange Pi WiringOP gpio tool."
        )

    result = subprocess.run(
        [gpio_command, *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=2.0,
        check=False,
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        raise RuntimeError(f"gpio {' '.join(args)} failed: {detail}")


def setup(gpio_command: str) -> None:
    run_gpio(["mode", str(LASER_PIN), "out"], gpio_command)


def write_pin(level: int, gpio_command: str) -> None:
    run_gpio(["write", str(LASER_PIN), str(level)], gpio_command)


def do_on(gpio_command: str) -> None:
    write_pin(0, gpio_command)
    print(f"ON  -> wPi {LASER_PIN} set to LOW")


def do_off(gpio_command: str) -> None:
    write_pin(1, gpio_command)
    print(f"OFF -> wPi {LASER_PIN} set to HIGH")


def do_pulse(duration: float, gpio_command: str) -> None:
    do_on(gpio_command)
    time.sleep(duration)
    do_off(gpio_command)
    print(f"PULSE done ({duration:.3f}s)")


def do_blink(times: int, interval: float, gpio_command: str) -> None:
    for _ in range(times):
        do_on(gpio_command)
        time.sleep(interval)
        do_off(gpio_command)
        time.sleep(interval)
    print(f"BLINK done (times={times}, interval={interval:.3f}s)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test laser pin wPi10")
    parser.add_argument("action", choices=["on", "off", "pulse", "blink"], help="Pin action")
    parser.add_argument("--duration", type=float, default=0.5, help="Pulse duration in seconds")
    parser.add_argument("--times", type=int, default=3, help="Blink times")
    parser.add_argument("--interval", type=float, default=0.2, help="Blink on/off interval in seconds")
    parser.add_argument("--gpio-command", default="gpio", help="GPIO command path or name")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        setup(args.gpio_command)
        do_off(args.gpio_command)

        if args.action == "on":
            do_on(args.gpio_command)
        elif args.action == "off":
            do_off(args.gpio_command)
        elif args.action == "pulse":
            do_pulse(args.duration, args.gpio_command)
        elif args.action == "blink":
            do_blink(args.times, args.interval, args.gpio_command)
        else:
            return 2

        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
