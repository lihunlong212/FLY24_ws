import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_path(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description():
    cartography_launch_args = {
        "use_rviz": "false",
    }

    uart_params = {
        "update_rate": 100.0,
        "source_frame": "map",
        "target_frame": "laser_link",
        "target_velocity_forwarding_auto_enable": True,
    }

    pid_params = {
        "control_frequency": 50.0,
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "kp_xy": 0.8,
        "ki_xy": 0.0,
        "kd_xy": 0.2,
        "kp_yaw": 1.0,
        "ki_yaw": 0.0,
        "kd_yaw": 0.2,
        "kp_z": 1.0,
        "ki_z": 0.0,
        "kd_z": 0.2,
        "max_linear_velocity": 33.0,
        "max_angular_velocity": 30.0,
        "max_vertical_velocity": 30.0,
    }

    route_params = {
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "position_tolerance_cm": 8.0,
        "yaw_tolerance_deg": 8.0,
        "height_tolerance_cm": 8.0,
        "spray_decision_timeout_sec": 1.5,
        "spray_data_stale_timeout_sec": 0.5,
        "spray_flash_on_sec": 0.3,
        "spray_flash_gap_sec": 0.3,
        "laser_on_command": 1,
        "laser_off_command": 2,
    }

    laser_params = {
        "pin": 10,
        "on_level": 0,
        "off_level": 1,
        "initial_off": True,
        "pulse_duration": 0.3,
        "command_topic": "/laser/cmd",
        "status_topic": "/laser/status",
    }

    camera_params = {
        "camera_device": "/dev/video2",
        "frame_width": 640,
        "frame_height": 480,
        "fps": 15.0,
        "window_name": "drone_camera_preview",
        "center_roi_width": 50,
        "center_roi_height": 50,
        "green_h_min": 25,
        "green_h_max": 100,
        "green_s_min": 20,
        "green_v_min": 40,
        "green_pixel_threshold": 100,
        "spray_allowed_topic": "/spray_allowed",
    }

    barcode_params = {
        "camera_device": "/dev/video0",
        "frame_width": 640,
        "frame_height": 480,
        "fps": 15.0,
        "barcode_topic": "/barcode_text",
        "show_preview": True,
        "window_name": "barcode_camera_preview",
        "publish_duplicates": False,
        "stop_after_first_publish": True,
    }

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py")),
        launch_arguments=cartography_launch_args.items(),
    )

    uart_node = Node(
        package="uart_to_stm32",
        executable="uart_to_stm32_node",
        name="uart_to_stm32",
        output="screen",
        parameters=[uart_params],
    )

    position_pid_controller_node = Node(
        package="pid_control_pkg",
        executable="position_pid_controller",
        name="position_pid_controller",
        output="screen",
        parameters=[pid_params],
    )

    route_node = Node(
        package="activity_control_pkg",
        executable="route_target_publisher_node",
        name="route_target_publisher",
        output="screen",
        parameters=[route_params],
    )

    laser_control_node = Node(
        package="laser_control_pkg",
        executable="laser_control_node",
        name="laser_control_node",
        output="screen",
        parameters=[laser_params],
    )

    drone_camera_node = Node(
        package="drone_camera_pkg",
        executable="drone_camera_node",
        name="drone_camera_node",
        output="screen",
        parameters=[camera_params],
    )

    barcode_camera_node = Node(
        package="barcode_camera_pkg",
        executable="barcode_camera_node",
        name="barcode_camera_node",
        output="screen",
        parameters=[barcode_params],
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_node,
        position_pid_controller_node,
        route_node,
        laser_control_node,
        drone_camera_node,
        barcode_camera_node,
    ])
