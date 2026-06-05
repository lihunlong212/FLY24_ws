#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class QRFineTuneNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_fine_tune_node")

        self.declare_parameter("input_prefix", "/qr")
        self.declare_parameter("output_topic", "/qr/fine_offset_body_cm")

        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("ema_alpha", 0.8)
        self.declare_parameter("deadband_ex", 0.02)
        self.declare_parameter("deadband_ey", 0.02)
        self.declare_parameter("max_step_cm", 2.0)

        self.declare_parameter("k_body_x_cm", 20.0)
        self.declare_parameter("k_body_y_cm", 0.0)
        self.declare_parameter("k_body_z_cm", 20.0)
        self.declare_parameter("max_cm", 15.0)
        self.declare_parameter("invert_body_x", True)
        self.declare_parameter("invert_body_y", False)
        self.declare_parameter("invert_body_z", True)
        self.declare_parameter("publish_zero_when_aligned", True)

        self.input_prefix = str(self.get_parameter("input_prefix").value).rstrip("/")
        if not self.input_prefix.startswith("/"):
            self.input_prefix = "/" + self.input_prefix

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.deadband_ex = float(self.get_parameter("deadband_ex").value)
        self.deadband_ey = float(self.get_parameter("deadband_ey").value)
        self.max_step_cm = float(self.get_parameter("max_step_cm").value)
        self.k_body_x_cm = float(self.get_parameter("k_body_x_cm").value)
        self.k_body_y_cm = float(self.get_parameter("k_body_y_cm").value)
        self.k_body_z_cm = float(self.get_parameter("k_body_z_cm").value)
        self.max_cm = float(self.get_parameter("max_cm").value)
        self.invert_body_x = bool(self.get_parameter("invert_body_x").value)
        self.invert_body_y = bool(self.get_parameter("invert_body_y").value)
        self.invert_body_z = bool(self.get_parameter("invert_body_z").value)
        self.publish_zero_when_aligned = bool(
            self.get_parameter("publish_zero_when_aligned").value
        )

        self._last_offset_norm: Optional[Point] = None
        self._aligned = False
        self._ex_f = 0.0
        self._ey_f = 0.0
        self._has_filt = False
        self._out_dx = 0.0
        self._out_dy = 0.0
        self._out_dz = 0.0

        self.offset_sub = self.create_subscription(
            Point, f"{self.input_prefix}/offset_norm", self._on_offset, 10
        )
        self.aligned_sub = self.create_subscription(
            Bool, f"{self.input_prefix}/aligned", self._on_aligned, 10
        )
        self.pub = self.create_publisher(Point, self.output_topic, 10)

        hz = max(1.0, self.publish_hz)
        self.timer = self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            "QRFineTuneNode started: "
            f"{self.input_prefix}/offset_norm,{self.input_prefix}/aligned -> {self.output_topic}"
        )

    def _on_offset(self, msg: Point) -> None:
        self._last_offset_norm = msg

    def _on_aligned(self, msg: Bool) -> None:
        self._aligned = bool(msg.data)

    def _publish_zero(self) -> None:
        out = Point()
        out.x = 0.0
        out.y = 0.0
        out.z = 0.0
        self.pub.publish(out)

    def _reset_output(self) -> None:
        self._has_filt = False
        self._out_dx = 0.0
        self._out_dy = 0.0
        self._out_dz = 0.0

    def _publish(self) -> None:
        if self.publish_zero_when_aligned and self._aligned:
            self._reset_output()
            self._publish_zero()
            return

        if self._last_offset_norm is None:
            self._publish_zero()
            return

        ex = float(self._last_offset_norm.x)
        ey = float(self._last_offset_norm.y)

        if math.fabs(ex) < self.deadband_ex:
            ex = 0.0
        if math.fabs(ey) < self.deadband_ey:
            ey = 0.0

        alpha = _clamp(self.ema_alpha, 0.0, 0.99)
        if not self._has_filt:
            self._ex_f = ex
            self._ey_f = ey
            self._has_filt = True
        else:
            self._ex_f = alpha * self._ex_f + (1.0 - alpha) * ex
            self._ey_f = alpha * self._ey_f + (1.0 - alpha) * ey

        desired_dx = self._ex_f * self.k_body_x_cm
        desired_dy = self._ex_f * self.k_body_y_cm
        desired_dz = self._ey_f * self.k_body_z_cm

        if self.invert_body_x:
            desired_dx = -desired_dx
        if self.invert_body_y:
            desired_dy = -desired_dy
        if self.invert_body_z:
            desired_dz = -desired_dz

        desired_dx = _clamp(desired_dx, -self.max_cm, self.max_cm)
        desired_dy = _clamp(desired_dy, -self.max_cm, self.max_cm)
        desired_dz = _clamp(desired_dz, -self.max_cm, self.max_cm)

        step = max(0.0, self.max_step_cm)
        self._out_dx += _clamp(desired_dx - self._out_dx, -step, step)
        self._out_dy += _clamp(desired_dy - self._out_dy, -step, step)
        self._out_dz += _clamp(desired_dz - self._out_dz, -step, step)

        if math.fabs(self._out_dx) < 0.1:
            self._out_dx = 0.0
        if math.fabs(self._out_dy) < 0.1:
            self._out_dy = 0.0
        if math.fabs(self._out_dz) < 0.1:
            self._out_dz = 0.0

        out = Point()
        out.x = self._out_dx
        out.y = self._out_dy
        out.z = self._out_dz
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRFineTuneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
