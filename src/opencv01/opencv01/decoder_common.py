#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from pyzbar import pyzbar
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

wiringpi = None
GPIO = None


class QRVisionNodeBase(Node):
    def __init__(
        self,
        node_name: str,
        topic_prefix: str,
        default_camera_device: str = "/dev/video0",
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("camera_device", default_camera_device)
        self.declare_parameter("eps_x", 0.40)
        self.declare_parameter("eps_y", 0.40)
        self.declare_parameter("eps_x_laser", 0.25)
        self.declare_parameter("stable_frames", 1)
        self.declare_parameter("enable_debug_image", False)
        self.declare_parameter("enable_gui", False)
        self.declare_parameter("decode_interval", 3)
        self.declare_parameter("laser_pin", -1)
        self.declare_parameter("qr_task_active_required", False)
        self.declare_parameter("laser_task_active_required", False)
        self.declare_parameter("laser_duration_sec", 1.0)
        self.declare_parameter("standalone_laser_once", True)

        self.camera_device = self.get_parameter("camera_device").value
        self.eps_x = float(self.get_parameter("eps_x").value)
        self.eps_y = float(self.get_parameter("eps_y").value)
        self.eps_x_laser = float(self.get_parameter("eps_x_laser").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)
        self.enable_debug = bool(self.get_parameter("enable_debug_image").value)
        self.enable_gui = bool(self.get_parameter("enable_gui").value)
        self.decode_interval = max(1, int(self.get_parameter("decode_interval").value))
        self.laser_pin = int(self.get_parameter("laser_pin").value)
        self.qr_task_active_required = bool(
            self.get_parameter("qr_task_active_required").value
        ) or bool(
            self.get_parameter("laser_task_active_required").value
        )
        self.laser_duration_sec = float(self.get_parameter("laser_duration_sec").value)
        self.standalone_laser_once = bool(self.get_parameter("standalone_laser_once").value)

        prefix = topic_prefix.strip().rstrip("/")
        self.topic_prefix = prefix if prefix.startswith("/") else "/" + prefix

        self.qr_id_pub = self.create_publisher(String, f"{self.topic_prefix}/id", 10)
        self.offset_pub = self.create_publisher(Point, f"{self.topic_prefix}/offset_norm", 10)
        self.aligned_pub = self.create_publisher(Bool, f"{self.topic_prefix}/aligned", 10)
        self.image_pub = self.create_publisher(Image, f"{self.topic_prefix}/debug_image", 10)
        self.qr_task_active_sub = self.create_subscription(
            Bool, "/qr_task_active", self._qr_task_active_callback, 10
        )
        self.laser_fire_sub = self.create_subscription(
            Bool, "/qr/fire_laser", self._laser_fire_callback, 10
        )

        self.camera_active = True
        self.qr_task_active = not self.qr_task_active_required

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_device}")
            raise RuntimeError("Camera open failed")

        self.stable_count = 0
        self.frame_count = 0
        self.previous_qr_data = ""
        self.laser_is_on = False
        self.laser_off_deadline = None
        self.standalone_laser_fired = False
        self.gpio_initialized = False
        self.should_show_window = False

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self._update_window_status()
        self._init_gpio()
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info(
            f"{node_name} started: camera={self.camera_device}, prefix={self.topic_prefix}, "
            f"laser_pin={self.laser_pin}, qr_task_active_required={self.qr_task_active_required}"
        )

    def _init_gpio(self) -> None:
        if self.gpio_initialized or self.laser_pin == -1:
            return

        global wiringpi, GPIO
        if wiringpi is None or GPIO is None:
            try:
                import wiringpi as wiringpi_module
                from wiringpi import GPIO as gpio_module

                wiringpi = wiringpi_module
                GPIO = gpio_module
            except ImportError:
                self.get_logger().warn("wiringpi is not installed; laser disabled.")
                self.laser_pin = -1
                return
            except BaseException as exc:
                self.get_logger().warn(f"wiringpi import failed; laser disabled: {exc}")
                self.laser_pin = -1
                return

        if wiringpi is None or GPIO is None:
            self.get_logger().warn("wiringpi is not installed; laser disabled.")
            self.laser_pin = -1
            return

        try:
            setup_result = wiringpi.wiringPiSetup()
        except BaseException as exc:
            self.get_logger().warn(f"GPIO setup failed; laser disabled: {exc}")
            self.laser_pin = -1
            return

        if setup_result == -1:
            self.get_logger().error("GPIO setup failed; laser disabled.")
            self.laser_pin = -1
            return

        try:
            wiringpi.pinMode(self.laser_pin, GPIO.OUTPUT)
            wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH)
        except Exception as exc:
            self.get_logger().error(f"GPIO init write failed; laser disabled: {exc}")
            self.laser_pin = -1
            return

        self.gpio_initialized = True
        self.get_logger().info(f"GPIO {self.laser_pin} initialized for laser output.")

    def _set_laser(self, enabled: bool) -> None:
        if self.laser_pin == -1:
            return
        if enabled and not self.gpio_initialized:
            self._init_gpio()
        if not self.gpio_initialized:
            return
        try:
            wiringpi.digitalWrite(self.laser_pin, GPIO.LOW if enabled else GPIO.HIGH)
            self.laser_is_on = enabled
        except Exception as exc:
            self.get_logger().error(f"Laser write failed: {exc}")

    def _fire_laser_for_duration(self) -> None:
        duration = max(0.0, self.laser_duration_sec)
        self._set_laser(True)
        if self.laser_is_on:
            self.laser_off_deadline = time.monotonic() + duration

    def _cancel_laser(self) -> None:
        self.laser_off_deadline = None
        self._set_laser(False)

    def _update_laser_timeout(self) -> None:
        if self.laser_off_deadline is None:
            return
        if time.monotonic() >= self.laser_off_deadline:
            self._cancel_laser()

    def _laser_fire_callback(self, msg: Bool) -> None:
        if not bool(msg.data) or not self.qr_task_active:
            self._cancel_laser()
            return
        self._fire_laser_for_duration()

    def _qr_task_active_callback(self, msg: Bool) -> None:
        self.qr_task_active = bool(msg.data)
        if not self.qr_task_active:
            self._cancel_laser()

    def _update_window_status(self) -> None:
        self.should_show_window = self.enable_gui and os.environ.get("DISPLAY") is not None
        if not self.should_show_window:
            try:
                cv2.destroyWindow(self.get_name())
            except Exception:
                pass

    def _on_parameter_change(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "enable_gui":
                self.enable_gui = bool(param.value)
                self._update_window_status()
            elif param.name == "decode_interval":
                self.decode_interval = max(1, int(param.value))
            elif param.name == "standalone_laser_once":
                self.standalone_laser_once = bool(param.value)
        return SetParametersResult(successful=True)

    def _maybe_fire_standalone_laser_once(self) -> None:
        if self.qr_task_active_required:
            return
        if not self.standalone_laser_once or self.standalone_laser_fired:
            return
        if self.laser_pin == -1:
            return

        self.standalone_laser_fired = True
        self.get_logger().info(
            f"Standalone QR aligned; firing laser once for {self.laser_duration_sec:.1f}s."
        )
        self._fire_laser_for_duration()

    def process_frame(self) -> None:
        self._update_laser_timeout()

        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        img_h, img_w = frame.shape[:2]
        img_cx = img_w / 2.0
        img_cy = img_h / 2.0

        if not self.qr_task_active:
            self.stable_count = 0
            self.aligned_pub.publish(Bool(data=False))
            return

        decoded_objects = []
        if self.frame_count % self.decode_interval == 0:
            decoded_objects = pyzbar.decode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        aligned = False
        found = False

        for obj in decoded_objects:
            found = True
            try:
                qr_data = obj.data.decode("utf-8")
            except UnicodeDecodeError:
                continue

            cx = obj.rect.left + obj.rect.width / 2.0
            cy = obj.rect.top + obj.rect.height / 2.0
            ex = (cx - img_cx) / img_cx
            ey = (cy - img_cy) / img_cy

            if abs(ex) < self.eps_x and abs(ey) < self.eps_y:
                self.stable_count += 1
            else:
                self.stable_count = 0

            aligned = self.stable_count >= self.stable_frames
            self.qr_id_pub.publish(String(data=qr_data))
            self.offset_pub.publish(Point(x=float(ex), y=float(ey), z=0.0))
            if aligned:
                self._maybe_fire_standalone_laser_once()

            if self.enable_debug or self.should_show_window:
                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)
                cv2.line(frame, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (255, 0, 0), 2)
                cv2.putText(
                    frame,
                    f"ID:{qr_data[:10]}",
                    (obj.rect.left, max(0, obj.rect.top - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )
            break

        if not found:
            self.stable_count = 0

        self.aligned_pub.publish(Bool(data=bool(aligned)))

        if self.enable_debug:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except Exception:
                pass

        if self.should_show_window:
            try:
                cv2.imshow(self.get_name(), frame)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().error(f"imshow failed: {exc}")
                self.should_show_window = False

    def destroy_node(self) -> None:
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
