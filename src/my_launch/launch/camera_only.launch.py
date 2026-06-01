from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([qr_decoder, qr_fine_tune])
