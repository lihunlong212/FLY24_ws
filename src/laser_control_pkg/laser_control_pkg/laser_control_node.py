from __future__ import annotations

import shutil
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class WiringPin:
    def __init__(self, pin: int, on_level: int, off_level: int) -> None:
        self.pin = pin
        self.on_level = on_level
        self.off_level = off_level
        self._gpio_cmd = shutil.which("gpio")
        self._wiringpi: Optional[object] = None

        if self._gpio_cmd is None:
            try:
                import wiringpi  # type: ignore

                self._wiringpi = wiringpi
                wiringpi.wiringPiSetup()
            except Exception:
                self._wiringpi = None

        self._set_mode_output()

    def _set_mode_output(self) -> None:
        if self._gpio_cmd is not None:
            subprocess.run([self._gpio_cmd, "mode", str(self.pin), "out"], check=True)
            return
        if self._wiringpi is not None:
            self._wiringpi.pinMode(self.pin, 1)
            return
        raise RuntimeError("No GPIO backend found. Install WiringOP 'gpio' or python wiringpi.")

    def write_enabled(self, enabled: bool) -> None:
        level = self.on_level if enabled else self.off_level
        if self._gpio_cmd is not None:
            subprocess.run([self._gpio_cmd, "write", str(self.pin), str(level)], check=True)
            return
        if self._wiringpi is not None:
            self._wiringpi.digitalWrite(self.pin, level)
            return
        raise RuntimeError("No GPIO backend is available")


class LaserControlNode(Node):
    def __init__(self) -> None:
        super().__init__("laser_control_node")

        self.declare_parameter("pin", 10)
        self.declare_parameter("on_level", 0)
        self.declare_parameter("off_level", 1)
        self.declare_parameter("initial_off", True)
        self.declare_parameter("pulse_duration", 1.0)
        self.declare_parameter("command_topic", "/laser/cmd")
        self.declare_parameter("status_topic", "/laser/status")

        self.pin = int(self.get_parameter("pin").value)
        self.on_level = int(self.get_parameter("on_level").value)
        self.off_level = int(self.get_parameter("off_level").value)
        self.pulse_duration = float(self.get_parameter("pulse_duration").value)
        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._enabled = False
        self._pulse_timer = None
        self._gpio = WiringPin(self.pin, self.on_level, self.off_level)

        self.status_pub = self.create_publisher(Bool, status_topic, 10)
        self.result_pub = self.create_publisher(String, "/laser/result", 10)
        self.cmd_sub = self.create_subscription(Int32, command_topic, self._cmd_callback, 10)

        if bool(self.get_parameter("initial_off").value):
            self._set_laser(False)

        self.get_logger().info(
            f"Laser control ready on WiringOP pin {self.pin}; low/on={self.on_level}, "
            f"high/off={self.off_level}. Command topic: {command_topic}"
        )

    def _cmd_callback(self, msg: Int32) -> None:
        command = int(msg.data)
        try:
            if command == 1:
                self._cancel_pulse_timer()
                self._set_laser(True)
                self._publish_result("1: laser enabled")
            elif command == 2:
                self._cancel_pulse_timer()
                self._set_laser(False)
                self._publish_result("2: laser disabled")
            elif command == 3:
                self._pulse()
            else:
                self._publish_result(f"unknown command {command}; use 1=on, 2=off, 3=pulse")
        except Exception as exc:
            self.get_logger().error(f"Failed to handle laser command {command}: {exc}")

    def _pulse(self) -> None:
        self._cancel_pulse_timer()
        self._set_laser(True)
        self._pulse_timer = self.create_timer(self.pulse_duration, self._finish_pulse)
        self._publish_result(f"3: laser pulse started for {self.pulse_duration:g} second(s)")

    def _finish_pulse(self) -> None:
        self._cancel_pulse_timer()
        self._set_laser(False)
        self._publish_result("3: laser pulse finished")

    def _cancel_pulse_timer(self) -> None:
        if self._pulse_timer is not None:
            self._pulse_timer.cancel()
            self.destroy_timer(self._pulse_timer)
            self._pulse_timer = None

    def _set_laser(self, enabled: bool) -> None:
        self._gpio.write_enabled(enabled)
        self._enabled = enabled
        msg = Bool()
        msg.data = enabled
        if rclpy.ok():
            self.status_pub.publish(msg)

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(text)

    def destroy_node(self) -> None:
        try:
            self._cancel_pulse_timer()
            self._set_laser(False)
        finally:
            super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaserControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
