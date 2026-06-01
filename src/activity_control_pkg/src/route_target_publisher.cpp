#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
constexpr int kSprayDecisionFrameCount = 3;

std::vector<Target> buildPlantProtectionRoute()
{
  return {
    Target{0.0, 0.0, 140.0, 0.0},

    Target{200.0, -50.0, 140.0, 0.0, true},
    Target{250.0, -50.0, 140.0, 0.0, true},

    Target{250.0, -100.0, 140.0, 0.0, true},
    Target{200.0, -100.0, 140.0, 0.0, true},

    Target{200.0, -150.0, 140.0, 0.0, true},
    Target{250.0, -150.0, 140.0, 0.0, true},

    Target{250.0, -200.0, 140.0, 0.0, true},
    Target{250.0, -250.0, 140.0, 0.0, true},
    Target{250.0, -300.0, 140.0, 0.0, true},
    Target{250.0, -350.0, 140.0, 0.0, true},

    Target{200.0, -350.0, 140.0, 0.0, true},
    Target{200.0, -300.0, 140.0, 0.0, true},
    Target{200.0, -250.0, 140.0, 0.0, true},
    Target{200.0, -200.0, 140.0, 0.0, true},

    Target{150.0, -200.0, 140.0, 0.0, true},
    Target{150.0, -250.0, 140.0, 0.0, true},
    Target{150.0, -300.0, 140.0, 0.0, true},
    Target{150.0, -350.0, 140.0, 0.0, true},

    Target{100.0, -350.0, 140.0, 0.0, true},
    Target{100.0, -300.0, 140.0, 0.0, true},
    Target{100.0, -250.0, 140.0, 0.0, true},
    Target{100.0, -200.0, 140.0, 0.0, true},

    Target{50.0, -200.0, 140.0, 0.0, true},
    Target{50.0, -250.0, 140.0, 0.0, true},
    Target{50.0, -300.0, 140.0, 0.0, true},
    Target{50.0, -350.0, 140.0, 0.0, true},

    Target{0.0, -350.0, 140.0, 0.0, true},
    Target{0.0, -300.0, 140.0, 0.0, true},
    Target{0.0, -250.0, 140.0, 0.0, true},
    Target{0.0, -200.0, 140.0, 0.0, true},
    Target{0.0, -200.0, 0.0, 0.0},

  };
}
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  spray_decision_timeout_sec_(0.0),
  spray_data_stale_timeout_sec_(0.0),
  spray_flash_on_sec_(0.0),
  spray_flash_gap_sec_(0.0),
  laser_on_command_(0),
  laser_off_command_(0),
  mission_complete_sent_(false),
  has_spray_allowed_(false),
  latest_spray_allowed_(false),
  spray_active_(false),
  spray_laser_step_(-1),
  spray_frame_count_(0),
  spray_seen_green_(false)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  spray_decision_timeout_sec_ = declare_parameter("spray_decision_timeout_sec", 1.5);
  spray_data_stale_timeout_sec_ = declare_parameter("spray_data_stale_timeout_sec", 0.5);
  spray_flash_on_sec_ = declare_parameter("spray_flash_on_sec", 0.3);
  spray_flash_gap_sec_ = declare_parameter("spray_flash_gap_sec", 0.3);
  laser_on_command_ = declare_parameter("laser_on_command", 1);
  laser_off_command_ = declare_parameter("laser_off_command", 2);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
  laser_cmd_pub_ = create_publisher<std_msgs::msg::Int32>("/laser/cmd", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  spray_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spray_allowed",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::sprayAllowedCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  loadSourceRoute();

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_,
    yaw_tol_deg_,
    height_tol_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Spray gating: decision_timeout=%.1fs stale=%.1fs flash_on=%.1fs flash_gap=%.1fs on_cmd=%d off_cmd=%d",
    spray_decision_timeout_sec_,
    spray_data_stale_timeout_sec_,
    spray_flash_on_sec_,
    spray_flash_gap_sec_,
    laser_on_command_,
    laser_off_command_);
}

void RouteTargetPublisherNode::loadSourceRoute()
{
  const std::vector<Target> route = buildPlantProtectionRoute();

  RCLCPP_INFO(get_logger(), "Loading source-defined waypoint route with %zu targets.", route.size());
  for (std::size_t index = 0; index < route.size(); ++index) {
    const Target target = route[index];
    targets_.push_back(target);
    RCLCPP_INFO(
      get_logger(),
      "Loaded auto waypoint %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f spray=%s",
      index + 1,
      route.size(),
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg,
      target.spray ? "true" : "false");
  }
  mission_complete_sent_ = false;
  current_idx_ = targets_.empty() ? std::numeric_limits<std::size_t>::max() : 0;
  RCLCPP_INFO(get_logger(), "Route is loaded. Publishing first waypoint immediately.");
  publishCurrent();
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  const bool was_completed =
    current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size();
  targets_.push_back(target);
  if (was_empty || was_completed) {
    mission_complete_sent_ = false;
    resetSprayState();
    current_idx_ = was_completed ? targets_.size() - 1 : 0;
    publishCurrent();
  }
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(getPublishedTarget(targets_[current_idx_]), current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg spray=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.spray ? "true" : "false",
    init_flag ? " (first)" : "");
}

Target RouteTargetPublisherNode::getPublishedTarget(const Target & target) const
{
  return target;
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::sprayAllowedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  latest_spray_allowed_ = msg->data;
  has_spray_allowed_ = true;
  last_spray_allowed_time_ = now();
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool z_ok = std::fabs(dz) <= height_tol_cm_;
  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  if (target.z_cm > 20.0) {
    if (current_idx_ == 0) {
      return z_ok;
    }
    return z_ok && xy_ok;
  }

  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  resetSprayState();
  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    if (!mission_complete_sent_ && mission_complete_pub_) {
      std_msgs::msg::Empty mission_complete_msg;
      mission_complete_pub_->publish(mission_complete_msg);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO(get_logger(), "All targets completed.");
  }
}

void RouteTargetPublisherNode::resetSprayState()
{
  spray_active_ = false;
  spray_laser_step_ = -1;
  spray_frame_count_ = 0;
  spray_seen_green_ = false;
  spray_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  spray_laser_step_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_sampled_spray_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void RouteTargetPublisherNode::publishLaserCommand(int command)
{
  std_msgs::msg::Int32 laser_msg;
  laser_msg.data = command;
  laser_cmd_pub_->publish(laser_msg);
}

bool RouteTargetPublisherNode::handleSprayTarget(const rclcpp::Time & now_time)
{
  if (!spray_active_) {
    spray_active_ = true;
    spray_laser_step_ = -1;
    spray_frame_count_ = 0;
    spray_seen_green_ = false;
    spray_start_time_ = now_time;
    last_sampled_spray_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(
      get_logger(),
      "Spray decision started for target %zu. Waiting for %d fresh color frames.",
      current_idx_,
      kSprayDecisionFrameCount);
  }

  if (spray_laser_step_ >= 0) {
    const double step_elapsed = (now_time - spray_laser_step_time_).seconds();
    if (spray_laser_step_ == 0 && step_elapsed >= spray_flash_on_sec_) {
      publishLaserCommand(laser_off_command_);
      spray_laser_step_ = 1;
      spray_laser_step_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Target %zu laser first flash off.", current_idx_);
    } else if (spray_laser_step_ == 1 && step_elapsed >= spray_flash_gap_sec_) {
      publishLaserCommand(laser_on_command_);
      spray_laser_step_ = 2;
      spray_laser_step_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Target %zu laser second flash on.", current_idx_);
    } else if (spray_laser_step_ == 2 && step_elapsed >= spray_flash_on_sec_) {
      publishLaserCommand(laser_off_command_);
      RCLCPP_INFO(get_logger(), "Target %zu laser sequence finished.", current_idx_);
      advanceToNextTarget();
    }
    return true;
  }

  const double elapsed = (now_time - spray_start_time_).seconds();
  const bool has_new_arrival_frame =
    has_spray_allowed_ &&
    last_spray_allowed_time_.nanoseconds() != 0 &&
    (last_spray_allowed_time_ - spray_start_time_).seconds() >= 0.0;
  const bool has_unsampled_frame =
    has_new_arrival_frame &&
    (last_spray_allowed_time_ - last_sampled_spray_time_).nanoseconds() > 0;

  if (!has_new_arrival_frame ||
    (now_time - last_spray_allowed_time_).seconds() > spray_data_stale_timeout_sec_)
  {
    if (elapsed >= spray_decision_timeout_sec_) {
      RCLCPP_WARN(
        get_logger(),
        "Only received %d/%d fresh /spray_allowed frames for target %zu after %.1fs. Skipping spray.",
        spray_frame_count_,
        kSprayDecisionFrameCount,
        current_idx_,
        elapsed);
      advanceToNextTarget();
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Waiting for fresh /spray_allowed for target %zu.",
      current_idx_);
    return true;
  }

  if (!has_unsampled_frame) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Waiting for next /spray_allowed frame for target %zu (%d/%d).",
      current_idx_,
      spray_frame_count_,
      kSprayDecisionFrameCount);
    return true;
  }

  ++spray_frame_count_;
  spray_seen_green_ = spray_seen_green_ || latest_spray_allowed_;
  last_sampled_spray_time_ = last_spray_allowed_time_;
  RCLCPP_INFO(
    get_logger(),
    "Spray color frame %d/%d for target %zu: %s.",
    spray_frame_count_,
    kSprayDecisionFrameCount,
    current_idx_,
    latest_spray_allowed_ ? "green" : "not green");

  if (spray_frame_count_ < kSprayDecisionFrameCount) {
    return true;
  }

  if (spray_seen_green_) {
    publishLaserCommand(laser_on_command_);
    spray_laser_step_ = 0;
    spray_laser_step_time_ = now_time;
    RCLCPP_INFO(
      get_logger(),
      "Target %zu has green in %d/%d sampled frames. Starting laser sequence: on %.1fs, off %.1fs, on %.1fs.",
      current_idx_,
      spray_frame_count_,
      kSprayDecisionFrameCount,
      spray_flash_on_sec_,
      spray_flash_gap_sec_,
      spray_flash_on_sec_);
    return true;
  }

  RCLCPP_INFO(
    get_logger(),
    "Target %zu has no green in %d sampled /spray_allowed frames. Skipping spray.",
    current_idx_,
    spray_frame_count_);
  advanceToNextTarget();
  return true;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "All targets completed. Keeping stop signal active.");
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  const rclcpp::Time now_time = now();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Current target %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f spray=%s",
    current_idx_,
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.spray ? "true" : "false");

  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    const double dx = target.x_cm - x_cm;
    const double dy = target.y_cm - y_cm;
    const double dz = target.z_cm - z_cm;
    const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
    RCLCPP_INFO(
      get_logger(),
      "Target %zu reached: pos_err=(%.1f, %.1f, %.1f)cm yaw_err=%.1fdeg current=(%.1f, %.1f, %.1f, %.1f)",
      current_idx_,
      dx,
      dy,
      dz,
      dyaw,
      x_cm,
      y_cm,
      z_cm,
      yaw_deg);
    if (target.spray && handleSprayTarget(now_time)) {
      return;
    }
    advanceToNextTarget();
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

}  // namespace activity_control_pkg
