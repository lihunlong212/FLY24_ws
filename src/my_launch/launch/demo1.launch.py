import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_path(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description():
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py"))
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("uart_to_stm32", "uart_to_stm32.launch.py"))
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_path("pid_control_pkg", "position_pid_controller.launch.py")
        )
    )

    route_target_publisher = Node(
        package="activity_control_pkg",
        executable="route_target_publisher_node",
        name="route_target_publisher",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            "fine_offset_limit_cm:=15.0",
            "-p",
            "fine_target_publish_hz:=10.0",
            "-p",
            "laser_fire_duration_sec:=1.0",
            "-p",
            "target_start_delay_sec:=5.0",
            "-p",
            "inventory_file_path:=src/activity_control_pkg/config/qr_inventory.csv",
        ],
    )

    bluetooth_node = Node(
        package="bluetooth",
        executable="bluetooth_node",
        name="bluetooth_node",
        output="screen",
        parameters=[
            {"port": "/dev/ttyS3"},
            {"baudrate": 9600},
        ],
    )

    qr_decoder = Node(
        package="opencv01",
        executable="qr_decoder_single",
        name="qr_decoder",
        output="screen",
        parameters=[
            {"camera_device": "/dev/video0"},
            {"eps_x": 0.40},
            {"eps_y": 0.40},
            {"eps_x_laser": 0.25},
            {"stable_frames": 1},
            {"enable_debug_image": False},
            {"enable_gui": False},
            {"decode_interval": 3},
            {"laser_pin": 10},
            {"laser_duration_sec": 1.0},
            {"qr_task_active_required": True},
            {"standalone_laser_once": False},
        ],
    )

    qr_fine_tune = Node(
        package="opencv01",
        executable="qr_fine_tune",
        name="qr_fine_tune",
        output="screen",
        parameters=[
            {"input_prefix": "/qr"},
            {"output_topic": "/qr/fine_offset_body_cm"},
            {"publish_hz": 5.0},
            {"ema_alpha": 0.9},
            {"deadband_ex": 0.05},
            {"deadband_ey": 0.05},
            {"max_step_cm": 1.0},
            {"k_body_x_cm": 20.0},
            {"k_body_y_cm": 0.0},
            {"k_body_z_cm": 20.0},
            {"max_cm": 15.0},
            {"invert_body_x": True},
            {"invert_body_y": False},
            {"invert_body_z": True},
            {"publish_zero_when_aligned": True},
        ],
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        route_target_publisher,
        bluetooth_node,
        TimerAction(period=1.0, actions=[qr_decoder, qr_fine_tune]),
    ])
