#!/usr/bin/env python3
"""
右侧相机二维码识别节点
发布话题前缀：/qr_right/*
"""

from __future__ import annotations

import rclpy

from opencv01.decoder_common import QRVisionNodeBase


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRVisionNodeBase(
        node_name="qr_vision_right",
        topic_prefix="/qr_right",
        default_camera_device="/dev/video0",
    )
    # 设置右侧引脚，应该用连接右侧激光器的引脚（实际是 Pin 10）
    node.set_parameters([rclpy.parameter.Parameter('laser_pin', rclpy.Parameter.Type.INTEGER, 10)])
    
    node.laser_pin = 10
    node._init_gpio()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


