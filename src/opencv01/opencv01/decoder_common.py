#!/usr/bin/env python3
"""
通用二维码识别节点基类 - 降频保护 + 动态窗口控制版
"""

from __future__ import annotations
import os
import cv2
from pyzbar import pyzbar
import threading
import wiringpi
from wiringpi import GPIO
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult # 【新增加】用于参数回调

from std_msgs.msg import String, Bool, UInt8
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class QRVisionNodeBase(Node):
    def __init__(self, node_name: str, topic_prefix: str, default_camera_device: str = "/dev/video0") -> None:
        super().__init__(node_name)

        # ---------------- 参数声明 ----------------
        self.declare_parameter("camera_device", default_camera_device)
        self.declare_parameter("eps_x", 0.40)
        self.declare_parameter("eps_y", 0.40)
        self.declare_parameter("eps_x_laser",0.25)
        self.declare_parameter("stable_frames", 1)
        self.declare_parameter("enable_debug_image", False)
        self.declare_parameter("enable_gui", False)  # 默认关闭 GUI
        self.declare_parameter("decode_interval", 3) # 默认每3帧解码一次
        
        self.declare_parameter("laser_pin", -1) 
        self.laser_pin = self.get_parameter("laser_pin").value


        
        # ---------------- 参数读取 ----------------
        self.camera_device = self.get_parameter("camera_device").value
        self.eps_x = self.get_parameter("eps_x").value
        self.eps_y = self.get_parameter("eps_y").value
        self.eps_x_laser = self.get_parameter("eps_x_laser").value
        self.stable_frames = self.get_parameter("stable_frames").value
        self.enable_debug = self.get_parameter("enable_debug_image").value
        self.enable_gui = self.get_parameter("enable_gui").value
        self.decode_interval = self.get_parameter("decode_interval").value

        # 【新增加】注册动态参数回调监听
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # 【新增加】初始化显示状态
        self._update_window_status()

        # 话题前缀处理
        prefix = topic_prefix.strip().rstrip("/")
        self.topic_prefix = prefix if prefix.startswith("/") else "/" + prefix

        # 发布器
        self.qr_id_pub = self.create_publisher(String, f"{self.topic_prefix}/id", 10)
        self.offset_pub = self.create_publisher(Point, f"{self.topic_prefix}/offset_norm", 10)
        self.aligned_pub = self.create_publisher(Bool, f"{self.topic_prefix}/aligned", 10)
        self.image_pub = self.create_publisher(Image, f"{self.topic_prefix}/debug_image", 10)

        if "/qr_right" in self.topic_prefix or "qr_right" in self.topic_prefix:
            self.camera_side_expected = 1  # 右摄像头期望的 camera_side
        elif "/qr_left" in self.topic_prefix or "qr_left" in self.topic_prefix:
            self.camera_side_expected = 2  # 左摄像头期望的 camera_side
        else:
            self.camera_side_expected = 0  # 默认都不激活
        
        self.current_target_camera_sub = self.create_subscription(
            UInt8, "/current_target_camera", self._camera_config_callback, 10
        )
        self.camera_active = True  # 默认激活（调试模式兼容）
        self._has_received_camera_config = False
        
        self.bridge = CvBridge()

        # 摄像头初始化
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_device}")
            raise RuntimeError("Camera open failed")

        self.stable_count = 0
        self.last_qr_id = ""
        self.frame_count = 0  # 【新增加】初始化帧计数器
        self.previous_qr_data = ""

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(f"{node_name} started on {self.camera_device}. Use 'ros2 param set' to toggle GUI.")
        self.gpio_initialized = False
        self._init_gpio()
        
    def _init_gpio(self):
        """初始化 GPIO 的逻辑"""
        # 如果还没初始化且引脚号合法，则进行初始化
        if not self.gpio_initialized and self.laser_pin != -1:
            if wiringpi.wiringPiSetup() == -1:
                self.get_logger().error("GPIO 初始化失败！请检查权限。")
                self.laser_pin = -1
            else:
                wiringpi.pinMode(self.laser_pin, GPIO.OUTPUT)
                wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH)
                self.gpio_initialized = True
                self.get_logger().info(f"GPIO {self.laser_pin} 初始化成功 (wPi编号)")

    def _fire_laser_worker(self):
        """线程工作函数：执行拉低0.5秒的操作"""
        if self.laser_pin == -1: return
        
        try:
            self.get_logger().info(f"==> 激光发射! (Pin {self.laser_pin})")
            wiringpi.digitalWrite(self.laser_pin, GPIO.LOW)  # 拉低
            import time
            time.sleep(0.5)                                 # 在子线程中延时，不影响主线程
            wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH) # 恢复高电平
            self.get_logger().info(f"==> 激光关闭")
        except Exception as e:
            self.get_logger().error(f"激光发射失败: {e}")
    def _camera_config_callback(self, msg):
        """当前航点的摄像头配置回调"""
        self._has_received_camera_config = True
        # 只有当航点指定的摄像头与自己匹配时才激活
        self.camera_active = (msg.data == self.camera_side_expected)
        if self.camera_active:
            self.get_logger().debug(f"摄像头激活: {self.get_name()}")
        else:
            self.get_logger().debug(f"摄像头停用: {self.get_name()}")
    # ------------------------------------------------------
    # 【新增加】动态控制逻辑
    # ------------------------------------------------------
    def _update_window_status(self):
        """根据当前参数和环境决定是否显示窗口"""
        has_display = os.environ.get("DISPLAY") is not None
        self.should_show_window = self.enable_gui and has_display
        
        if not self.should_show_window:
            try:
                cv2.destroyWindow(self.get_name())
            except Exception:
                pass

    def _on_parameter_change(self, params):
        """当终端修改参数时触发"""
        for param in params:
            if param.name == "enable_gui":
                self.enable_gui = param.value
                self._update_window_status()
                self.get_logger().info(f"GUI status changed to: {self.enable_gui}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------
    # 主处理循环
    # ------------------------------------------------------
    def process_frame(self) -> None:
        if self._has_received_camera_config and not self.camera_active:
            ret,_ = self.cap.read()
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1 # 【新增加】计数

        img_h, img_w = frame.shape[:2]
        img_cx, img_cy = img_w / 2.0, img_h / 2.0

        # 【新增加】降频逻辑判断
        decoded_objects = []
        if self.frame_count % self.decode_interval == 0:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            decoded_objects = pyzbar.decode(frame_rgb)

        aligned = False
        found = False
        laser_ready = False

        for obj in decoded_objects:
            found = True
            try:
                qr_data = obj.data.decode("utf-8")
                self.last_qr_id = qr_data
            except:
                continue

            cx = obj.rect.left + obj.rect.width / 2.0
            cy = obj.rect.top + obj.rect.height / 2.0
            ex = (cx - img_cx) / img_cx
            ey = (cy - img_cy) / img_cy

            if abs(ex) < self.eps_x and abs(ey) < self.eps_y: # 因为我是一帧对准，所以我激光的判断逻辑写在哪都没有问题，读者如是修改成多帧对准，就要好好考虑了
                self.stable_count += 1
                laser_ready = abs(ex) < self.eps_x_laser
            else:
                self.stable_count = 0
                laser_ready = False

            aligned = self.stable_count >= self.stable_frames
            
            if laser_ready and qr_data != self.previous_qr_data:
                # 开启新线程去打激光，主线程立刻返回继续下一帧识别
                threading.Thread(target=self._fire_laser_worker, daemon=True).start()
                self.previous_qr_data = qr_data

            # 发布 ID 和 偏差
            self.qr_id_pub.publish(String(data=qr_data))
            self.offset_pub.publish(Point(x=float(ex), y=float(ey), z=0.0))

            # 绘制调试信息
            if self.enable_debug or self.should_show_window:
                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)
                cv2.line(frame, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (255, 0, 0), 2)
                cv2.putText(frame, f"ID:{qr_data[:10]}", (obj.rect.left, max(0, obj.rect.top - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            break

        if not found:
            self.stable_count = 0

        self.aligned_pub.publish(Bool(data=bool(aligned)))
        
        if aligned:
            self.get_logger().info(f"✓ QR已对准中心 (stable_count={self.stable_count})")
        else:
            if found:
                self.get_logger().debug(f"QR未对准 (stable_count={self.stable_count}/{self.stable_frames})")

        # 图像发布
        if self.enable_debug:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except:
                pass

        # 【新增加】窗口显示控制
        if self.should_show_window:
            try:
                cv2.imshow(self.get_name(), frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"imshow failed: {e}")
                self.should_show_window = False

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

# #!/usr/bin/env python3
# """
# 通用二维码识别节点基类 - 降频保护 + 动态窗口控制版
# """

# from __future__ import annotations
# import os
# import cv2
# from pyzbar import pyzbar
# import threading
# import wiringpi
# from wiringpi import GPIO
# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.msg import SetParametersResult # 【新增加】用于参数回调

# from std_msgs.msg import String, Bool
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge


# class QRVisionNodeBase(Node):
#     def __init__(self, node_name: str, topic_prefix: str, default_camera_device: str = "/dev/video0") -> None:
#         super().__init__(node_name)

#         # ---------------- 参数声明 ----------------
#         self.declare_parameter("camera_device", default_camera_device)
#         self.declare_parameter("eps_x", 0.40)
#         self.declare_parameter("eps_y", 0.40)
#         self.declare_parameter("eps_x_laser",0.25)
#         self.declare_parameter("stable_frames", 1)
#         self.declare_parameter("enable_debug_image", True)
#         self.declare_parameter("enable_gui", False)  # 默认关闭 GUI
#         self.declare_parameter("decode_interval", 3) # 默认每3帧解码一次
        
#         self.declare_parameter("laser_pin", -1) 
#         self.laser_pin = self.get_parameter("laser_pin").value


        
#         # ---------------- 参数读取 ----------------
#         self.camera_device = self.get_parameter("camera_device").value
#         self.eps_x = self.get_parameter("eps_x").value
#         self.eps_y = self.get_parameter("eps_y").value
#         self.eps_x_laser = self.get_parameter("eps_x_laser").value
#         self.stable_frames = self.get_parameter("stable_frames").value
#         self.enable_debug = self.get_parameter("enable_debug_image").value
#         self.enable_gui = self.get_parameter("enable_gui").value
#         self.decode_interval = self.get_parameter("decode_interval").value

#         # 【新增加】注册动态参数回调监听
#         self.add_on_set_parameters_callback(self._on_parameter_change)

#         # 【新增加】初始化显示状态
#         self._update_window_status()

#         # 话题前缀处理
#         prefix = topic_prefix.strip().rstrip("/")
#         self.topic_prefix = prefix if prefix.startswith("/") else "/" + prefix

#         # 发布器
#         self.qr_id_pub = self.create_publisher(String, f"{self.topic_prefix}/id", 10)
#         self.offset_pub = self.create_publisher(Point, f"{self.topic_prefix}/offset_norm", 10)
#         self.aligned_pub = self.create_publisher(Bool, f"{self.topic_prefix}/aligned", 10)
#         self.image_pub = self.create_publisher(Image, f"{self.topic_prefix}/debug_image", 10)

#         self.bridge = CvBridge()

#         # 摄像头初始化
#         self.cap = cv2.VideoCapture(self.camera_device)
#         if not self.cap.isOpened():
#             self.get_logger().error(f"Failed to open camera {self.camera_device}")
#             raise RuntimeError("Camera open failed")

#         self.stable_count = 0
#         self.last_qr_id = ""
#         self.frame_count = 0  # 【新增加】初始化帧计数器
#         self.previous_qr_data = ""

#         self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
#         self.get_logger().info(f"{node_name} started on {self.camera_device}. Use 'ros2 param set' to toggle GUI.")
#         self.gpio_initialized = False
#         self._init_gpio()
        
#     def _init_gpio(self):
#         """初始化 GPIO 的逻辑"""
#         # 如果还没初始化且引脚号合法，则进行初始化
#         if not self.gpio_initialized and self.laser_pin != -1:
#             if wiringpi.wiringPiSetup() == -1:
#                 self.get_logger().error("GPIO 初始化失败！请检查权限。")
#                 self.laser_pin = -1
#             else:
#                 wiringpi.pinMode(self.laser_pin, GPIO.OUTPUT)
#                 wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH)
#                 self.gpio_initialized = True
#                 self.get_logger().info(f"GPIO {self.laser_pin} 初始化成功 (wPi编号)")

#     def _fire_laser_worker(self):
#         """线程工作函数：执行拉低0.5秒的操作"""
#         if self.laser_pin == -1: return
        
#         try:
#             self.get_logger().info(f"==> 激光发射! (Pin {self.laser_pin})")
#             wiringpi.digitalWrite(self.laser_pin, GPIO.LOW)  # 拉低
#             import time
#             time.sleep(0.5)                                 # 在子线程中延时，不影响主线程
#             wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH) # 恢复高电平
#             self.get_logger().info(f"==> 激光关闭")
#         except Exception as e:
#             self.get_logger().error(f"激光发射失败: {e}")
        
#     # ------------------------------------------------------
#     # 【新增加】动态控制逻辑
#     # ------------------------------------------------------
#     def _update_window_status(self):
#         """根据当前参数和环境决定是否显示窗口"""
#         has_display = os.environ.get("DISPLAY") is not None
#         self.should_show_window = self.enable_gui and has_display
        
#         if not self.should_show_window:
#             try:
#                 cv2.destroyWindow(self.get_name())
#             except Exception:
#                 pass

#     def _on_parameter_change(self, params):
#         """当终端修改参数时触发"""
#         for param in params:
#             if param.name == "enable_gui":
#                 self.enable_gui = param.value
#                 self._update_window_status()
#                 self.get_logger().info(f"GUI status changed to: {self.enable_gui}")
#         return SetParametersResult(successful=True)

#     # ------------------------------------------------------
#     # 主处理循环
#     # ------------------------------------------------------
#     def process_frame(self) -> None:
#         ret, frame = self.cap.read()
#         if not ret:
#             return

#         self.frame_count += 1 # 【新增加】计数

#         img_h, img_w = frame.shape[:2]
#         img_cx, img_cy = img_w / 2.0, img_h / 2.0

#         # 【新增加】降频逻辑判断
#         decoded_objects = []
#         if self.frame_count % self.decode_interval == 0:
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             decoded_objects = pyzbar.decode(frame_rgb)

#         aligned = False
#         found = False
#         laser_ready = False

#         for obj in decoded_objects:
#             found = True
#             try:
#                 qr_data = obj.data.decode("utf-8")
#                 self.last_qr_id = qr_data
#             except:
#                 continue

#             cx = obj.rect.left + obj.rect.width / 2.0
#             cy = obj.rect.top + obj.rect.height / 2.0
#             ex = (cx - img_cx) / img_cx
#             ey = (cy - img_cy) / img_cy

#             if abs(ex) < self.eps_x and abs(ey) < self.eps_y:
#                 self.stable_count += 1
#             else:
#                 self.stable_count = 0
                
#             laser_ready = abs(ex) < self.eps_x_laser

#             aligned = self.stable_count >= self.stable_frames
            
#             if laser_ready and qr_data != self.previous_qr_data:
#                 # 开启新线程去打激光，主线程立刻返回继续下一帧识别
#                 threading.Thread(target=self._fire_laser_worker, daemon=True).start()
#                 self.previous_qr_data = qr_data

#             # 发布 ID 和 偏差
#             self.qr_id_pub.publish(String(data=qr_data))
#             self.offset_pub.publish(Point(x=float(ex), y=float(ey), z=0.0))

#             # 绘制调试信息
#             if self.enable_debug or self.should_show_window:
#                 cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)
#                 cv2.line(frame, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (255, 0, 0), 2)
#                 cv2.putText(frame, f"ID:{qr_data[:10]}", (obj.rect.left, max(0, obj.rect.top - 10)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
#             break

#         if not found:
#             self.stable_count = 0

#         self.aligned_pub.publish(Bool(data=bool(aligned)))
        
#         if aligned:
#             self.get_logger().info(f"✓ QR已对准中心 (stable_count={self.stable_count})")
#         else:
#             if found:
#                 self.get_logger().debug(f"QR未对准 (stable_count={self.stable_count}/{self.stable_frames})")

#         # 图像发布
#         if self.enable_debug:
#             try:
#                 self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
#             except:
#                 pass

#         # 【新增加】窗口显示控制
#         if self.should_show_window:
#             try:
#                 cv2.imshow(self.get_name(), frame)
#                 cv2.waitKey(1)
#             except Exception as e:
#                 self.get_logger().error(f"imshow failed: {e}")
#                 self.should_show_window = False

#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()



