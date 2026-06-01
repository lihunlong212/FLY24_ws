#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <clocale>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

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

const std::vector<Target> & demoRoute()
{
  // {x_cm, y_cm, z_cm, yaw_deg, need_qr_laser, invert_xy_velocity, yaw_only}
  // yaw_only can be set manually; 0 <-> 180 deg transitions are detected automatically.
  static const std::vector<Target> route = {
    {0.0, 0.0, 130.0, 0.0, false, false, false},
    {100.0, 0.0, 130.0, 0.0, false, false, false},
    {100.0, 0.0, 130.0, 180.0, false, false, true},
    {100.0, -100.0, 130.0, 180.0, false, true, false},
    {0.0, -100.0, 130.0, 180.0, false, true, false},
    {0.0, 0.0, 130.0, 180.0, false, true, false},
    {0.0, 0.0, 0.0, 180.0, false, true, false},

    // {170.0, 0.0, 130.0, 0.0, true, false, false},
    // {230.0, 0.0, 130.0, 0.0, true, false, false},
    // {240.0, 0.0, 75.0, 0.0, true, false, false},
    // {170.0, 0.0, 83.0, 0.0, true, false, false},
    // {120.0, 0.0, 83.0, 0.0, true, false, false},
    // {0.0, 0.0, 83.0, 0.0, false, false, false},
    // {0.0, -96.0, 87.0, 0.0, false, false, false},
    // {170.0, -96.0, 87.0, 0.0, false, false, false},
    // {235.0, -96.0, 87.0, 0.0, false, false, false},
    // {240.0, -96.0, 130.0, 0.0, false, false, false},
    // {170.0, -96.0, 130.0, 0.0, false, false, false},
    // {120.0, -96.0, 130.0, 0.0, false, false, false},
    // {0.0, -96.0, 130.0, 0.0, false, false, false},
    // {0.0, -96.0, 4.0, 0.0, false, false, false},
  };
  return route;
}
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  enable_visual_takeover_ = declare_parameter("enable_visual_takeover", false);
  visual_takeover_distance_cm_ = declare_parameter("visual_takeover_distance_cm", 10.0);
  fine_offset_limit_cm_ = declare_parameter("fine_offset_limit_cm", 12.0);
  fine_target_publish_hz_ = declare_parameter("fine_target_publish_hz", 5.0);
  enable_visual_align_for_low_z_targets_ =
    declare_parameter("enable_visual_align_for_low_z_targets", false);
  visual_align_z_threshold_cm_ = declare_parameter("visual_align_z_threshold_cm", 20.0);
  laser_fire_duration_sec_ = declare_parameter("laser_fire_duration_sec", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", qos);
  qr_task_active_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_task_active", qos);
  qr_laser_fire_pub_ = create_publisher<std_msgs::msg::Bool>("/qr/fire_laser", qos);
  qr_inventory_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/qr/inventory_data", qos);

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  qr_id_sub_ = create_subscription<std_msgs::msg::String>(
    "/qr/id", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrIdCallback, this, std::placeholders::_1));
  qr_aligned_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/qr/aligned", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrAlignedCallback, this, std::placeholders::_1));
  qr_fine_offset_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/qr/fine_offset_body_cm", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrFineOffsetCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  RCLCPP_INFO(
    get_logger(), "RouteTargetPublisher: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(), laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Visual takeover: enable=%s near=%.1fcm fine_limit=%.1fcm low_z_enable=%s z_th=%.1fcm",
    enable_visual_takeover_ ? "true" : "false",
    visual_takeover_distance_cm_,
    fine_offset_limit_cm_,
    enable_visual_align_for_low_z_targets_ ? "true" : "false",
    visual_align_z_threshold_cm_);
  publishQrTaskActive(false);
  publishLaserFire(false);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
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
    Target target = targets_[current_idx_];
    target.yaw_only = target.yaw_only || isYawOnlyTransition(current_idx_);
    publishTarget(target, current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data = {
    static_cast<float>(target.x_cm),
    static_cast<float>(target.y_cm),
    static_cast<float>(target.z_cm),
    static_cast<float>(target.yaw_deg),
    target.invert_xy_velocity ? 1.0f : 0.0f,
    target.yaw_only ? 1.0f : 0.0f,
  };
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg qr_laser=%s invert_xy=%s yaw_only=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.require_visual_align ? "true" : "false",
    target.invert_xy_velocity ? "true" : "false",
    target.yaw_only ? "true" : "false",
    init_flag ? " (first)" : "");
}

void RouteTargetPublisherNode::publishQrTaskActive(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  qr_task_active_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishLaserFire(bool fire)
{
  std_msgs::msg::Bool msg;
  msg.data = fire;
  qr_laser_fire_pub_->publish(msg);
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::qrIdCallback(const std_msgs::msg::String::SharedPtr msg)
{
  latest_qr_id_ = msg->data;
  has_latest_qr_id_ = true;
}

void RouteTargetPublisherNode::qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  qr_aligned_ = msg->data;
  has_qr_aligned_ = true;
}

void RouteTargetPublisherNode::qrFineOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  qr_fine_offset_body_cm_ = *msg;
  has_qr_fine_offset_ = true;
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
      get_logger(), *get_clock(), 2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
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

  if (target.yaw_only) {
    return z_ok && yaw_ok;
  }

  if (target.z_cm > 20.0) {
    if (current_idx_ == 0) {
      return z_ok;
    }
    if (std::fabs(target.yaw_deg) > 1.0 || target.invert_xy_velocity) {
      return z_ok && xy_ok && yaw_ok;
    }
    return z_ok && xy_ok;
  }
  return z_ok && xy_ok && yaw_ok;
}

bool RouteTargetPublisherNode::isNearXY(const Target & target, double x_cm, double y_cm) const
{
  return std::hypot(target.x_cm - x_cm, target.y_cm - y_cm) <= visual_takeover_distance_cm_;
}

bool RouteTargetPublisherNode::isYawOnlyTransition(std::size_t index) const
{
  if (index == 0 || index >= targets_.size()) {
    return false;
  }

  const double yaw_delta = std::fabs(
    normalizeAngleDeg(targets_[index].yaw_deg - targets_[index - 1].yaw_deg));
  return yaw_delta > 90.0;
}

void RouteTargetPublisherNode::resetQrStateForTarget()
{
  latest_qr_id_.clear();
  has_latest_qr_id_ = false;
  qr_aligned_ = false;
  has_qr_aligned_ = false;
  qr_fine_offset_body_cm_ = geometry_msgs::msg::Point{};
  has_qr_fine_offset_ = false;
  qr_code_recorded_ = false;
  laser_fire_active_ = false;
  fine_anchor_valid_ = false;
  publishLaserFire(false);
}

std::string RouteTargetPublisherNode::currentQrSlotName() const
{
  const std::size_t index = qr_inventory_.size();
  if (index >= 24) {
    return "overflow";
  }

  const char row = static_cast<char>('A' + static_cast<int>(index / 6));
  const int col = static_cast<int>(index % 6) + 1;
  return std::string(1, row) + std::to_string(col);
}

bool RouteTargetPublisherNode::parseQrCodeByte(const std::string & text, std::uint8_t & value) const
{
  std::string trimmed;
  for (char ch : text) {
    if (!std::isspace(static_cast<unsigned char>(ch))) {
      trimmed.push_back(ch);
    }
  }
  if (trimmed.empty()) {
    return false;
  }

  try {
    std::size_t parsed_chars = 0;
    const unsigned long parsed = std::stoul(trimmed, &parsed_chars, 0);
    if (parsed_chars == trimmed.size() && parsed <= 255UL) {
      value = static_cast<std::uint8_t>(parsed);
      return true;
    }
  } catch (const std::exception &) {
  }

  value = static_cast<std::uint8_t>(trimmed.front());
  return true;
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  publishQrTaskActive(false);
  publishLaserFire(false);
  qr_sequence_active_ = false;
  laser_fire_active_ = false;
  fine_anchor_valid_ = false;

  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
  }
}

bool RouteTargetPublisherNode::handleQrTarget(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm)
{
  if (!qr_sequence_active_) {
    qr_sequence_active_ = true;
    resetQrStateForTarget();
    publishQrTaskActive(true);
    RCLCPP_INFO(
      get_logger(), "QR task started for slot %s at target %zu.",
      currentQrSlotName().c_str(), current_idx_);
  } else {
    publishQrTaskActive(true);
  }

  if (!qr_code_recorded_) {
    if (!has_latest_qr_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for QR code at slot %s.", currentQrSlotName().c_str());
      return true;
    }

    std::uint8_t value = 0;
    if (!parseQrCodeByte(latest_qr_id_, value)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "QR code is empty or invalid at slot %s.", currentQrSlotName().c_str());
      return true;
    }

    const std::string slot_name = currentQrSlotName();

    if (qr_inventory_.size() < 24) {
      qr_inventory_.push_back(value);
    } else {
      qr_inventory_.back() = value;
    }

    std_msgs::msg::UInt8MultiArray inventory_msg;
    inventory_msg.data = qr_inventory_;
    qr_inventory_pub_->publish(inventory_msg);

    qr_code_recorded_ = true;
    RCLCPP_INFO(
      get_logger(), "Recorded QR %s: raw='%s' value=%u (%zu/24).",
      slot_name.c_str(), latest_qr_id_.c_str(),
      static_cast<unsigned int>(value), qr_inventory_.size());
  }

  if (!has_qr_aligned_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /qr/aligned.");
    return true;
  }

  if (qr_aligned_) {
    if (!laser_fire_active_) {
      laser_fire_active_ = true;
      laser_fire_start_time_ = now();
      publishLaserFire(true);
      RCLCPP_INFO(get_logger(), "QR aligned. Laser firing for %.1fs.", laser_fire_duration_sec_);
      return true;
    }

    if ((now() - laser_fire_start_time_).seconds() < laser_fire_duration_sec_) {
      publishLaserFire(true);
      return true;
    }

    publishLaserFire(false);
    RCLCPP_INFO(get_logger(), "Laser done. Advancing to next target.");
    advanceToNextTarget();
    return true;
  }

  if (!has_qr_fine_offset_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for /qr/fine_offset_body_cm.");
    return true;
  }

  const double hz = std::max(1.0, fine_target_publish_hz_);
  const double min_period = 1.0 / hz;
  const rclcpp::Time now_time = now();
  if (
    fine_last_publish_time_.nanoseconds() != 0 &&
    (now_time - fine_last_publish_time_).seconds() < min_period)
  {
    return true;
  }

  const double body_dx_cm =
    std::clamp(qr_fine_offset_body_cm_.x, -fine_offset_limit_cm_, fine_offset_limit_cm_);
  const double body_dy_cm =
    std::clamp(qr_fine_offset_body_cm_.y, -fine_offset_limit_cm_, fine_offset_limit_cm_);
  const double body_dz_cm =
    std::clamp(qr_fine_offset_body_cm_.z, -fine_offset_limit_cm_, fine_offset_limit_cm_);

  if (!fine_anchor_valid_ || fine_anchor_target_idx_ != current_idx_) {
    fine_anchor_valid_ = true;
    fine_anchor_target_idx_ = current_idx_;
    fine_anchor_x_cm_ = x_cm;
    fine_anchor_y_cm_ = y_cm;
    fine_anchor_z_cm_ = z_cm;
  }

  Target fine_target = target;
  fine_target.x_cm = fine_anchor_x_cm_ + body_dx_cm;
  fine_target.y_cm = fine_anchor_y_cm_ + body_dy_cm;
  fine_target.z_cm = fine_anchor_z_cm_ + body_dz_cm;
  publishTarget(fine_target, false);
  fine_last_publish_time_ = now_time;
  return true;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    publishQrTaskActive(false);
    publishLaserFire(false);
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "All targets completed.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  Target target = targets_[current_idx_];
  target.yaw_only = target.yaw_only || isYawOnlyTransition(current_idx_);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Current target %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg);

  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    const bool require_visual_align = target.require_visual_align ||
      (enable_visual_align_for_low_z_targets_ && target.z_cm <= visual_align_z_threshold_cm_);

    if (require_visual_align && handleQrTarget(target, x_cm, y_cm, z_cm)) {
      return;
    }

    advanceToNextTarget();
    return;
  }

  if (!qr_sequence_active_) {
    publishQrTaskActive(false);
    publishLaserFire(false);
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

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false),
  next_target_index_(0)
{
  std::setlocale(LC_ALL, "");

  add_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();

  const auto & route = demoRoute();
  if (!route.empty()) {
    route_node_->addTarget(route.front());
    next_target_index_ = 1;
    add_timer_->reset();
    started_ = true;
    RCLCPP_INFO(get_logger(), "Route test started with %zu configured targets.", route.size());
  }
}

void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  const auto & route = demoRoute();
  if (next_target_index_ >= route.size()) {
    add_timer_->cancel();
    RCLCPP_INFO(get_logger(), "All configured route targets have been added.");
    return;
  }

  const Target & target = route[next_target_index_];
  route_node_->addTarget(target);
  RCLCPP_INFO(
    get_logger(),
    "Added route target %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f qr_laser=%s invert_xy=%s yaw_only=%s",
    next_target_index_ + 1,
    route.size(),
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.require_visual_align ? "true" : "false",
    target.invert_xy_velocity ? "true" : "false",
    target.yaw_only ? "true" : "false");
  ++next_target_index_;
}

}  // namespace activity_control_pkg
