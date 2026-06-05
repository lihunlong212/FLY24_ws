import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SIDE_IMAGE_TOPIC = "/warehouse_inventory/side_camera/image_raw"
BARCODE_TOPIC = "/warehouse_inventory/barcode_value"
BARCODE_CANDIDATE_TOPIC = "/warehouse_inventory/barcode_candidate"
BARCODE_OVERLAY_TOPIC = "/warehouse_inventory/barcode_overlay"
FINE_DATA_TOPIC = "/fine_data"
BARCODE_ACTIVE_TOPIC = "/barcode_task_active"

VISUAL_TARGET_OFFSET_X_PX = -75.0
VISUAL_TARGET_OFFSET_Y_PX = 20.0
VISUAL_PIXEL_DEADZONE = 5.0


def _launch_path(package_name: str, filename: str) -> str:
    share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(share, "launch", filename)


def _workspace_root() -> str:
    share = FindPackageShare(package="my_launch").find("my_launch")
    install_marker = os.sep + "install" + os.sep
    if install_marker in share:
        return share.split(install_marker, 1)[0]

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if os.path.basename(launch_dir) == "launch":
        return os.path.abspath(os.path.join(launch_dir, "..", "..", ".."))
    return os.getcwd()


def _task_log_dir(task_name: str) -> str:
    log_dir = os.path.join(_workspace_root(), "mylog", task_name)
    os.makedirs(log_dir, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = log_dir
    return log_dir


def generate_launch_description() -> LaunchDescription:
    use_viewer = LaunchConfiguration("use_viewer")
    task_log_dir = _task_log_dir("warehouse_inventory")

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py"))
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("uart_to_stm32", "uart_to_stm32.launch.py"))
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_path("pid_control_pkg", "position_pid_controller.launch.py")
        ),
        launch_arguments={
            "visual_mapping_mode": "right_side_camera",
            "visual_kp_x": "0.08",
            "visual_kd_x": "0.01",
            "visual_kp_z": "0.06",
            "visual_kd_z": "0.01",
            "visual_target_offset_x_px": str(VISUAL_TARGET_OFFSET_X_PX),
            "visual_target_offset_y_px": str(VISUAL_TARGET_OFFSET_Y_PX),
            "visual_pixel_deadzone": str(VISUAL_PIXEL_DEADZONE),
            "visual_max_xy_velocity": "20.0",
            "visual_max_z_velocity": "18.0",
            "visual_data_timeout_sec": "0.5",
        }.items(),
    )

    camera_source = Node(
        package="visual_pkg",
        executable="camera_source_node",
        name="warehouse_side_camera_source",
        output="screen",
        parameters=[{
            "camera_device": "/dev/video0",
            "width": 640,
            "height": 480,
            "camera_fps": 30,
            "publish_fps": 30.0,
            "frame_id": "warehouse_side_camera",
            "image_topic": SIDE_IMAGE_TOPIC,
        }],
    )

    barcode_reader = Node(
        package="activity_control_pkg",
        executable="barcode_reader_node",
        name="warehouse_qr_reader_node",
        output="screen",
        parameters=[{
            "image_topic": SIDE_IMAGE_TOPIC,
            "barcode_topic": BARCODE_TOPIC,
            "candidate_topic": BARCODE_CANDIDATE_TOPIC,
            "overlay_topic": BARCODE_OVERLAY_TOPIC,
            "fine_data_topic": FINE_DATA_TOPIC,
            "active_topic": BARCODE_ACTIVE_TOPIC,
            "task_active_required": True,
            "publish_fine_data": True,
            "overlay_target_offset_x_px": VISUAL_TARGET_OFFSET_X_PX,
            "overlay_target_offset_y_px": VISUAL_TARGET_OFFSET_Y_PX,
            "overlay_pixel_deadzone": VISUAL_PIXEL_DEADZONE,
            "stable_count": 2,
            "republish_period_sec": 0.2,
        }],
    )

    side_camera_viewer = Node(
        package="activity_control_pkg",
        executable="side_camera_viewer_node",
        name="warehouse_side_camera_viewer_node",
        output="screen",
        parameters=[{
            "image_topic": BARCODE_OVERLAY_TOPIC,
            "candidate_topic": BARCODE_CANDIDATE_TOPIC,
            "barcode_topic": BARCODE_TOPIC,
            "fine_data_topic": FINE_DATA_TOPIC,
            "target_offset_x_px": VISUAL_TARGET_OFFSET_X_PX,
            "target_offset_y_px": VISUAL_TARGET_OFFSET_Y_PX,
            "pixel_deadzone": VISUAL_PIXEL_DEADZONE,
            "window_name": "Warehouse side camera",
            "display_width": 960,
        }],
        condition=IfCondition(use_viewer),
    )

    warehouse_task = Node(
        package="activity_control_pkg",
        executable="warehouse_inventory_task_node",
        name="warehouse_inventory_task_node",
        output="screen",
        parameters=[{
            "mission_mode": "inventory",
            "map_frame": "map",
            "laser_link_frame": "laser_link",
            "output_topic": "/target_position",
            "height_topic": "/height",
            "position_tolerance_cm": 5.0,
            "height_tolerance_cm": 4.0,
            "yaw_tolerance_deg": 3.0,
            "log_waypoint_targets": False,
            "timer_period_sec": 0.05,
            "mission_mode_topic": "/mission_mode",
            "barcode_active_topic": BARCODE_ACTIVE_TOPIC,
            "barcode_topic": BARCODE_TOPIC,
            "fine_data_topic": FINE_DATA_TOPIC,
            "laser_pin": 10,
            "laser_on_sec": 1.0,
            "inventory_event_topic": "/qr/inventory_event",
            "target_event_topic": "/qr/target_event",
        }],
    )

    bluetooth_node = Node(
        package="bluetooth",
        executable="bluetooth_node",
        name="bluetooth_node",
        output="screen",
        parameters=[{
            "port": "/dev/ttyS3",
            "baudrate": 115200,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", task_log_dir),
        LogInfo(msg=f"Task log directory: {task_log_dir}"),
        DeclareLaunchArgument(
            "use_viewer",
            default_value="false",
            description="Show OpenCV side-camera debug window.",
        ),
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        camera_source,
        barcode_reader,
        side_camera_viewer,
        warehouse_task,
        bluetooth_node,
    ])
