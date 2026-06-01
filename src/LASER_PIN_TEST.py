#!/usr/bin/env python3
"""
Minimal Orange Pi laser pin test (wiringPi numbering).

Default pins in this repo:
- right laser: wPi 10
- left laser : wPi 13

Examples:
  sudo python3 LASER_PIN_TEST.py on
  sudo python3 LASER_PIN_TEST.py off
  sudo python3 LASER_PIN_TEST.py pulse
  sudo python3 LASER_PIN_TEST.py blink --times 5 --interval 0.3
"""

from __future__ import annotations

import argparse
import sys
import time

import wiringpi
from wiringpi import GPIO


RIGHT_PIN = 10
LEFT_PIN = 13
PINS = (RIGHT_PIN, LEFT_PIN)


def setup() -> None:
    if wiringpi.wiringPiSetup() == -1:
        raise RuntimeError("wiringPi setup failed. Run with sudo and check wiringPi installation.")
    for pin in PINS:
        wiringpi.pinMode(pin, GPIO.OUTPUT)


def resolve_pins(target: str) -> tuple[int, ...]:
    if target == "both":
        return PINS
    if target == "right":
        return (RIGHT_PIN,)
    if target == "left":
        return (LEFT_PIN,)
    raise ValueError(f"invalid target: {target}")


def write_pins(level: int, target: str) -> None:
    for pin in resolve_pins(target):
        wiringpi.digitalWrite(pin, level)


def do_on(target: str) -> None:
    write_pins(GPIO.LOW, target)
    print(f"ON  -> {target} set to LOW")


def do_off(target: str) -> None:
    write_pins(GPIO.HIGH, target)
    print(f"OFF -> {target} set to HIGH")


def do_pulse(duration: float, target: str) -> None:
    do_on(target)
    time.sleep(duration)
    do_off(target)
    print(f"PULSE done ({duration:.3f}s, target={target})")


def do_blink(times: int, interval: float, target: str) -> None:
    for _ in range(times):
        do_on(target)
        time.sleep(interval)
        do_off(target)
        time.sleep(interval)
    print(f"BLINK done (times={times}, interval={interval:.3f}s, target={target})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test two laser pins: wPi10 and wPi13")
    parser.add_argument("action", choices=["on", "off", "pulse", "blink"], help="Pin action")
    parser.add_argument("--target", choices=["left", "right", "both"], default="both", help="Choose which laser to control")
    parser.add_argument("--duration", type=float, default=0.5, help="Pulse duration in seconds")
    parser.add_argument("--times", type=int, default=3, help="Blink times")
    parser.add_argument("--interval", type=float, default=0.2, help="Blink on/off interval in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        setup()
        # Keep default state as OFF (HIGH) when script starts.
        do_off(args.target)

        if args.action == "on":
            do_on(args.target)
        elif args.action == "off":
            do_off(args.target)
        elif args.action == "pulse":
            do_pulse(args.duration, args.target)
        elif args.action == "blink":
            do_blink(args.times, args.interval, args.target)
        else:
            return 2

        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
