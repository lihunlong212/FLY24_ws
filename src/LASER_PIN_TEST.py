#!/usr/bin/env python3
"""
Minimal Orange Pi laser pin test (wiringPi numbering).

Default laser pin in this repo:
- laser: wPi 10

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


LASER_PIN = 10


def setup() -> None:
    if wiringpi.wiringPiSetup() == -1:
        raise RuntimeError("wiringPi setup failed. Run with sudo and check wiringPi installation.")
    wiringpi.pinMode(LASER_PIN, GPIO.OUTPUT)


def write_pin(level: int) -> None:
    wiringpi.digitalWrite(LASER_PIN, level)


def do_on() -> None:
    write_pin(GPIO.LOW)
    print(f"ON  -> pin {LASER_PIN} set to LOW")


def do_off() -> None:
    write_pin(GPIO.HIGH)
    print(f"OFF -> pin {LASER_PIN} set to HIGH")


def do_pulse(duration: float) -> None:
    do_on()
    time.sleep(duration)
    do_off()
    print(f"PULSE done ({duration:.3f}s)")


def do_blink(times: int, interval: float) -> None:
    for _ in range(times):
        do_on()
        time.sleep(interval)
        do_off()
        time.sleep(interval)
    print(f"BLINK done (times={times}, interval={interval:.3f}s)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test laser pin wPi10")
    parser.add_argument("action", choices=["on", "off", "pulse", "blink"], help="Pin action")
    parser.add_argument("--duration", type=float, default=0.5, help="Pulse duration in seconds")
    parser.add_argument("--times", type=int, default=3, help="Blink times")
    parser.add_argument("--interval", type=float, default=0.2, help="Blink on/off interval in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        setup()
        # Keep default state as OFF (HIGH) when script starts.
        do_off()

        if args.action == "on":
            do_on()
        elif args.action == "off":
            do_off()
        elif args.action == "pulse":
            do_pulse(args.duration)
        elif args.action == "blink":
            do_blink(args.times, args.interval)
        else:
            return 2

        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
