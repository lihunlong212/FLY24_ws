from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="laser_control_pkg",
            executable="laser_control_node",
            name="laser_control_node",
            output="screen",
            parameters=[
                {
                    "pin": 10,
                    "on_level": 0,
                    "off_level": 1,
                    "initial_off": True,
                    "pulse_duration": 0.3,
                    "command_topic": "/laser/cmd",
                    "status_topic": "/laser/status",
                }
            ],
        )
    ])
