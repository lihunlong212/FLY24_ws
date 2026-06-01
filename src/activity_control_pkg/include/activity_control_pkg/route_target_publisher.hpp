#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool spray = false;
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  std::size_t currentIndex() const;
  std::size_t size() const;

private:
  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);
  Target getPublishedTarget(const Target & target) const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void sprayAllowedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void advanceToNextTarget();
  void loadSourceRoute();
  void resetSprayState();
  bool handleSprayTarget(const rclcpp::Time & now_time);
  void publishLaserCommand(int command);

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr laser_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spray_allowed_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  double spray_decision_timeout_sec_;
  double spray_data_stale_timeout_sec_;
  double spray_flash_on_sec_;
  double spray_flash_gap_sec_;
  int laser_on_command_;
  int laser_off_command_;

  bool mission_complete_sent_;

  bool has_spray_allowed_;
  bool latest_spray_allowed_;
  rclcpp::Time last_spray_allowed_time_;

  bool spray_active_;
  int spray_laser_step_;
  int spray_frame_count_;
  bool spray_seen_green_;
  rclcpp::Time spray_start_time_;
  rclcpp::Time spray_laser_step_time_;
  rclcpp::Time last_sampled_spray_time_;
};

}  // namespace activity_control_pkg
