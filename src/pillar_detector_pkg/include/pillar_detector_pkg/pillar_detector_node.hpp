#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace pillar_detector_pkg
{

struct Detection
{
  double x_m;
  double y_m;
};

struct Cluster
{
  double x_m;
  double y_m;
  int votes;
};

class PillarDetectorNode : public rclcpp::Node
{
public:
  explicit PillarDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan);
  std::vector<Detection> detectInFrame(const sensor_msgs::msg::LaserScan & scan);
  bool findBestPillar(const std::vector<Detection> & detections, Cluster & pillar) const;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishPillar(const Cluster & pillar);

  double map_x_min_m_;
  double map_x_max_m_;
  double map_y_min_m_;
  double map_y_max_m_;

  int min_pts_per_group_;
  double group_dist_m_;
  double min_pillar_separation_m_;

  int accumulation_frames_;
  double cluster_merge_dist_m_;
  int min_votes_;

  int frame_count_;
  bool done_;
  bool ranges_precomputed_;
  std::size_t last_in_bounds_count_;
  std::size_t last_group_count_;
  std::vector<double> max_range_per_idx_;
  std::vector<Detection> accumulated_;
  mutable std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_pub_;
};

}  // namespace pillar_detector_pkg
