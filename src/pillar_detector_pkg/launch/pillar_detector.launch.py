from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pillar_detector_pkg",
            executable="pillar_detector_node",
            name="pillar_detector",
            output="screen",
            parameters=[{
                # 雷达输入话题。
                "scan_topic": "/scan",
                # 杆子坐标输出话题，格式为 Float32MultiArray [x_m, y_m]。
                "output_topic": "/detected_pillar",
                # 搜索杆子的 map 坐标矩形区域，单位 m。
                "map_x_min_m": 0.5,
                "map_x_max_m": 2.5,
                # y 是负方向区域，所以这里表示只搜索 -2.5m 到 -0.5m 之间。
                "map_y_min_m": -2.5,
                "map_y_max_m": -0.5,
                # 单帧聚类距离，单位 m；相邻雷达点距离小于该值会被归为同一组。
                "group_dist_m": 0.25,
                # 三脚架小柱子很细，单帧可能只有 1~2 个雷达点，所以这里放宽到 1。
                "min_pts_per_group": 1,
                # 三脚架有多个小柱子，需要保留多个候选，最后由代码选择最近的那个。
                "min_pillar_separation_m": 0.08,
                # 连续累计多少帧雷达数据后再输出最终杆子坐标。
                "accumulation_frames": 20,
                # 多帧候选点合并距离，单位 m；距离小于该值认为是同一个杆子的多次观测。
                "cluster_merge_dist_m": 0.20,
                # 20 帧里至少命中这么多票，才确认杆子有效。
                "min_votes": 8,
            }],
        )
    ])
