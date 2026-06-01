#!/usr/bin/env python3
import cv2
import numpy as np
from pyzbar import pyzbar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# 新增导入
from std_msgs.msg import String
from geometry_msgs.msg import Point

class QRDecoder(Node):
    def __init__(self):
        super().__init__('qr_decoder')
        self.bridge = CvBridge()
        # 发布处理后的图像话题
        self.image_pub = self.create_publisher(Image, "/qr_processed_image", 10)
        # 新增：发布二维码数据的话题
        self.qr_data_pub = self.create_publisher(String, "/qr_code_data", 10)
        # 新增：发布二维码坐标的话题
        self.qr_position_pub = self.create_publisher(Point, "/qr_code_position", 10)
        
    def decode_qr_codes(self, image):
        # 使用pyzbar解码二维码
        decoded_objects = pyzbar.decode(image)
        
        for obj in decoded_objects:
            # 提取二维码数据
            data = obj.data.decode("utf-8")
            self.get_logger().info(f"QR Code Data: {data}")
            
            # 新增：发布二维码数据到话题
            qr_data_msg = String()
            qr_data_msg.data = data
            self.qr_data_pub.publish(qr_data_msg)
            
            # 新增：发布二维码中心坐标到话题
            center_x = obj.rect.left + obj.rect.width / 2
            center_y = obj.rect.top + obj.rect.height / 2
            qr_position_msg = Point()
            qr_position_msg.x = float(center_x)
            qr_position_msg.y = float(center_y)
            qr_position_msg.z = 0  # z轴设为0，因为我们处理的是2D图像
            self.qr_position_pub.publish(qr_position_msg)
            
            # 打印二维码像素坐标
            self.get_logger().info(f"QR Code Position: ({center_x}, {center_y})")
            
            # 获取二维码位置
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points
            
            # 绘制二维码边界
            n = len(hull)
            for j in range(0, n):
                cv2.line(image, hull[j], hull[(j+1) % n], (255, 0, 0), 3)
            
            # 在图像上显示二维码数据
            x = obj.rect.left
            y = obj.rect.top
            cv2.putText(image, data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        return image, decoded_objects
    
def main():
    # 初始化ros节点
    rclpy.init()
    # 初始化类
    qr_decoder = QRDecoder()
    try:
        #打开摄像头
        cap = cv2.VideoCapture("/dev/video0")
        #判断是否成功打开摄像头
        if not cap.isOpened():
            qr_decoder.get_logger().error("无法打开摄像头")
            exit(-1)
        
        # 持续读取和处理视频流
        while rclpy.ok():
            #读取摄像头一帧
            ret , frame = cap.read()
            if not ret:
                qr_decoder.get_logger().warn("无法获取帧")
                break
                
            #将图像转化为rgb格式
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #调用解码二维码函数
            processed_frame, decoded_objects = qr_decoder.decode_qr_codes(frame_rgb.copy())
            
            # 将处理后的图像转换回BGR格式用于显示
            processed_frame_bgr = cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)
            
            # 发布处理后的图像到ROS话题
            try:
                ros_image = qr_decoder.bridge.cv2_to_imgmsg(processed_frame_bgr, "bgr8")
                qr_decoder.image_pub.publish(ros_image)
            except CvBridgeError as e:
                qr_decoder.get_logger().error(f"cv_bridge error: {e}")
            
            #输出视频
            cv2.imshow("QR Code Decoder", processed_frame_bgr)
            
            # 按ESC键退出
            if cv2.waitKey(1) & 0xFF == 27:
                break
            
            # 处理 ROS2 回调（必须）
            rclpy.spin_once(qr_decoder, timeout_sec=0.001)
                
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
        
    except KeyboardInterrupt:
        qr_decoder.get_logger().info("Shutting down")
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
    finally:
        qr_decoder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




