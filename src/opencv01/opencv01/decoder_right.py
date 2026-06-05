#!/usr/bin/env python3
from __future__ import annotations

import rclpy

from opencv01.decoder_common import QRVisionNodeBase


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRVisionNodeBase(
        node_name="qr_vision",
        topic_prefix="/qr",
        default_camera_device="/dev/video0",
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
