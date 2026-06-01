import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch_path(package_name: str, filename: str) -> str:
    return os.path.join(get_package_share_directory(package_name), "launch", filename)


def generate_launch_description() -> LaunchDescription:
    route_params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "position_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "height_tolerance_cm": 6.0,
        "spray_decision_timeout_sec": 1.5,
        "spray_data_stale_timeout_sec": 0.5,
        "spray_flash_on_sec": 0.3,
        "spray_flash_gap_sec": 0.3,
        "laser_on_command": 1,
        "laser_off_command": 2,
    }

    uart_params = {
        "update_rate": 100.0,
        "source_frame": "map",
        "target_frame": "laser_link",
        "target_velocity_forwarding_auto_enable": True,
    }

    cartography_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py")),
        launch_arguments={"use_rviz": "false"}.items(),
    )
    pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_path("pid_control_pkg", "position_pid_controller.launch.py")
        )
    )

    uart_node = Node(
        package="uart_to_stm32",
        executable="uart_to_stm32_node",
        name="uart_to_stm32",
        output="screen",
        parameters=[uart_params],
    )
    route_node = Node(
        package="activity_control_pkg",
        executable="route_target_publisher_node",
        name="route_target_publisher",
        output="screen",
        parameters=[route_params],
    )

    return LaunchDescription([
        cartography_launch,
        uart_node,
        pid_launch,
        route_node,
    ])
