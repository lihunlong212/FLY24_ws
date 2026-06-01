from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    uart_params = {
        "update_rate": 100.0,
        "source_frame": "map",
        "target_frame": "laser_link",
        "target_velocity_forwarding_auto_enable": True,
    }

    return LaunchDescription([
        Node(
            package="uart_to_stm32",
            executable="uart_to_stm32_node",
            name="uart_to_stm32",
            parameters=[uart_params],
            output="screen",
        )
    ])
