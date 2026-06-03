#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <clocale>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
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
constexpr std::uint8_t kInventoryMode = 1;
constexpr std::uint8_t kTargetMode = 2;

Target target(
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg,
  bool require_visual_align = false,
  bool invert_xy_velocity = false,
  bool yaw_only = false,
  const std::string & slot_name = "",
  std::uint8_t slot_code = 0)
{
  return Target{
    x_cm,
    y_cm,
    z_cm,
    yaw_deg,
    require_visual_align,
    invert_xy_velocity,
    yaw_only,
    slot_name,
    slot_code};
}

const std::vector<Target> & shelfTargets()
{
  static const std::vector<Target> slots = {
    target(175.0, -40.0, 128.0, 0.0, true, false, false, "A1", 0xA1),
    target(125.0, -40.0, 128.0, 0.0, true, false, false, "A2", 0xA2),
    target(75.0, -40.0, 128.0, 0.0, true, false, false, "A3", 0xA3),
    target(175.0, -40.0, 83.0, 0.0, true, false, false, "A4", 0xA4),
    target(125.0, -40.0, 83.0, 0.0, true, false, false, "A5", 0xA5),
    target(75.0, -40.0, 83.0, 0.0, true, false, false, "A6", 0xA6),

    target(75.0, -110.0, 128.0, 180.0, true, true, false, "B1", 0xB1),
    target(125.0, -110.0, 128.0, 180.0, true, true, false, "B2", 0xB2),
    target(175.0, -110.0, 128.0, 180.0, true, true, false, "B3", 0xB3),
    target(75.0, -110.0, 83.0, 180.0, true, true, false, "B4", 0xB4),
    target(125.0, -110.0, 83.0, 180.0, true, true, false, "B5", 0xB5),
    target(175.0, -110.0, 83.0, 180.0, true, true, false, "B6", 0xB6),

    target(175.0, -245.0, 128.0, 0.0, true, false, false, "C1", 0xC1),
    target(125.0, -245.0, 128.0, 0.0, true, false, false, "C2", 0xC2),
    target(75.0, -245.0, 128.0, 0.0, true, false, false, "C3", 0xC3),
    target(175.0, -245.0, 83.0, 0.0, true, false, false, "C4", 0xC4),
    target(125.0, -245.0, 83.0, 0.0, true, false, false, "C5", 0xC5),
    target(75.0, -245.0, 83.0, 0.0, true, false, false, "C6", 0xC6),

    target(75.0, -305.0, 128.0, 180.0, true, true, false, "D1", 0xD1),
    target(125.0, -305.0, 128.0, 180.0, true, true, false, "D2", 0xD2),
    target(175.0, -305.0, 128.0, 180.0, true, true, false, "D3", 0xD3),
    target(175.0, -305.0, 83.0, 180.0, true, true, false, "D4", 0xD4),
    target(125.0, -305.0, 83.0, 180.0, true, true, false, "D5", 0xD5),
    target(75.0, -305.0, 83.0, 180.0, true, true, false, "D6", 0xD6),
  };
  return slots;
}

void appendSlots(std::vector<Target> & route, std::size_t first, std::size_t last)
{
  const auto & slots = shelfTargets();
  for (std::size_t i = first; i < last && i < slots.size(); ++i) {
    route.push_back(slots[i]);
  }
}

std::vector<Target> inventoryRoute()
{
  std::vector<Target> route;
  route.reserve(34);

  route.push_back(target(0.0, 0.0, 128.0, 0.0));

  appendSlots(route, 0, 6);
  route.push_back(target(-10.0, -40.0, 83.0, 0.0));
  route.push_back(target(-10.0, -40.0, 83.0, 180.0, false, false, true));
  route.push_back(target(-10.0, -110.0, 83.0, 180.0, false, true, false));

  appendSlots(route, 6, 12);
  route.push_back(target(175.0, -110.0, 83.0, 0.0, false, false, true));

  appendSlots(route, 12, 18);
  route.push_back(target(-10.0, -245.0, 83.0, 0.0));
  route.push_back(target(-10.0, -245.0, 83.0, 180.0, false, false, true));
  route.push_back(target(-10.0, -305.0, 83.0, 180.0, false, true, false));

  appendSlots(route, 18, 24);
  route.push_back(target(250.0, -350.0, 83.0, 180.0, false, true, false));
  route.push_back(target(250.0, -350.0, 0.0, 180.0, false, true, false));

  return route;
}

std::string trim(const std::string & value)
{
  auto begin = value.begin();
  while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }

  auto end = value.end();
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }

  return std::string(begin, end);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }
  return fields;
}
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  mission_state_(MissionState::WAIT_FOR_MODE),
  has_height_(false),
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  inventory_file_path_ =
    declare_parameter("inventory_file_path", "src/activity_control_pkg/config/qr_inventory.csv");
  target_start_delay_sec_ = declare_parameter("target_start_delay_sec", 5.0);

  fine_offset_limit_cm_ = declare_parameter("fine_offset_limit_cm", 12.0);
  fine_target_publish_hz_ = declare_parameter("fine_target_publish_hz", 5.0);
  laser_fire_duration_sec_ = declare_parameter("laser_fire_duration_sec", 1.0);

  inventory_records_ = makeEmptyInventoryRecords();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", qos);
  qr_task_active_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_task_active", qos);
  qr_laser_fire_pub_ = create_publisher<std_msgs::msg::Bool>("/qr/fire_laser", qos);
  qr_inventory_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/qr/inventory_data", qos);
  qr_inventory_event_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/qr/inventory_event", rclcpp::QoS(10));
  qr_target_event_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/qr/target_event", rclcpp::QoS(10));

  mission_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/mission_mode", qos,
    std::bind(&RouteTargetPublisherNode::missionModeCallback, this, std::placeholders::_1));
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
    get_logger(), "RouteTargetPublisher waiting for /mission_mode (1=inventory, 2=target).");
  RCLCPP_INFO(
    get_logger(), "Frames: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(), laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm, inventory=%s",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_, inventory_file_path_.c_str());

  publishQrTaskActive(false);
  publishLaserFire(false);
}

void RouteTargetPublisherNode::missionModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (mission_state_ != MissionState::WAIT_FOR_MODE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Mission mode already latched, ignoring new mode %u.",
      static_cast<unsigned int>(msg->data));
    return;
  }

  if (msg->data == kInventoryMode) {
    startInventoryMission();
  } else if (msg->data == kTargetMode) {
    startTargetMissionPreflight();
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for mission mode 1 or 2, got %u.", static_cast<unsigned int>(msg->data));
  }
}

void RouteTargetPublisherNode::startInventoryMission()
{
  inventory_records_ = makeEmptyInventoryRecords();
  writeInventoryCsv();
  publishInventoryData();
  has_target_qr_value_ = false;
  RCLCPP_INFO(get_logger(), "Mission mode 1 latched: inventory flight.");
  setRoute(inventoryRoute(), MissionState::INVENTORY_FLIGHT);
}

void RouteTargetPublisherNode::startTargetMissionPreflight()
{
  targets_.clear();
  current_idx_ = std::numeric_limits<std::size_t>::max();
  mission_state_ = MissionState::TARGET_WAIT_QR;
  resetQrStateForTarget();
  has_target_qr_value_ = false;
  publishQrTaskActive(true);
  publishLaserFire(false);
  RCLCPP_INFO(get_logger(), "Mission mode 2 latched: waiting for hand-held target QR.");
}

void RouteTargetPublisherNode::setRoute(const std::vector<Target> & route, MissionState state)
{
  targets_ = route;
  mission_state_ = state;
  qr_sequence_active_ = false;
  qr_code_recorded_ = false;
  laser_fire_active_ = false;
  inventory_event_sent_ = false;
  fine_anchor_valid_ = false;
  publishQrTaskActive(false);
  publishLaserFire(false);

  if (targets_.empty()) {
    current_idx_ = std::numeric_limits<std::size_t>::max();
    mission_state_ = MissionState::COMPLETE;
    return;
  }

  current_idx_ = 0;
  publishCurrent();
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
    if (mission_state_ == MissionState::WAIT_FOR_MODE) {
      mission_state_ = MissionState::INVENTORY_FLIGHT;
    }
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
    Target current = targets_[current_idx_];
    current.yaw_only = current.yaw_only || isYawOnlyTransition(current_idx_);
    publishTarget(current, current_idx_ == 0);
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
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg qr=%s invert_xy=%s yaw_only=%s slot=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.require_visual_align ? "true" : "false",
    target.invert_xy_velocity ? "true" : "false",
    target.yaw_only ? "true" : "false",
    target.slot_name.empty() ? "-" : target.slot_name.c_str(),
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

void RouteTargetPublisherNode::publishInventoryData()
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.data.reserve(inventory_records_.size());
  for (const auto & record : inventory_records_) {
    msg.data.push_back(record.has_qr_value ? record.qr_value : 0);
  }
  qr_inventory_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishInventoryEvent(
  const Target & target,
  std::uint8_t qr_value)
{
  if (target.slot_code == 0 || !isValidQrValue(qr_value)) {
    return;
  }

  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {target.slot_code, qr_value};
  qr_inventory_event_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "Inventory event: [0xAA, %s, 0x%02X, 0xFF].",
    slotCodeToString(target.slot_code).c_str(), static_cast<unsigned int>(qr_value));
}

void RouteTargetPublisherNode::publishTargetEvent(
  const InventoryRecord & record,
  std::uint8_t qr_value)
{
  if (record.slot_code == 0 || !isValidQrValue(qr_value)) {
    return;
  }

  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {record.slot_code, qr_value};
  qr_target_event_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "Target event: [0xAA, %s, 0x%02X, 0xFF, 0xFF].",
    slotCodeToString(record.slot_code).c_str(), static_cast<unsigned int>(qr_value));
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

bool RouteTargetPublisherNode::isYawOnlyTransition(std::size_t index) const
{
  if (index == 0 || index >= targets_.size()) {
    return false;
  }

  const Target & previous = targets_[index - 1];
  const Target & current = targets_[index];
  const double yaw_delta = std::fabs(
    normalizeAngleDeg(current.yaw_deg - previous.yaw_deg));
  const double dxy = std::hypot(current.x_cm - previous.x_cm, current.y_cm - previous.y_cm);
  const double dz = std::fabs(current.z_cm - previous.z_cm);
  return yaw_delta > 90.0 && dxy <= 1.0 && dz <= 1.0;
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
  inventory_event_sent_ = false;
  recorded_qr_value_ = 0;
  fine_anchor_valid_ = false;
  fine_last_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  publishLaserFire(false);
}

bool RouteTargetPublisherNode::parseQrCodeByte(
  const std::string & text,
  std::uint8_t & value) const
{
  const std::string trimmed = trim(text);
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

  if (trimmed.size() == 1) {
    value = static_cast<std::uint8_t>(trimmed.front());
    return true;
  }
  return false;
}

bool RouteTargetPublisherNode::isValidQrValue(std::uint8_t value) const
{
  return value >= 1 && value <= 24;
}

bool RouteTargetPublisherNode::recordInventoryValue(
  const Target & target,
  std::uint8_t qr_value)
{
  if (target.slot_code == 0 || !isValidQrValue(qr_value)) {
    return false;
  }

  for (auto & record : inventory_records_) {
    if (record.slot_code == target.slot_code) {
      record.qr_value = qr_value;
      record.has_qr_value = true;
      writeInventoryCsv();
      publishInventoryData();
      return true;
    }
  }
  return false;
}

bool RouteTargetPublisherNode::writeInventoryCsv()
{
  try {
    const std::filesystem::path path(inventory_file_path_);
    const auto parent = path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out) {
      RCLCPP_WARN(get_logger(), "Failed to open inventory CSV for writing: %s", inventory_file_path_.c_str());
      return false;
    }

    out << "slot,slot_code,x_cm,y_cm,z_cm,yaw_deg,invert_xy_velocity,qr_value\n";
    out << std::fixed << std::setprecision(1);
    for (const auto & record : inventory_records_) {
      out << record.slot_name << ','
          << slotCodeToString(record.slot_code) << ','
          << record.target.x_cm << ','
          << record.target.y_cm << ','
          << record.target.z_cm << ','
          << record.target.yaw_deg << ','
          << (record.target.invert_xy_velocity ? 1 : 0) << ',';
      if (record.has_qr_value) {
        out << static_cast<unsigned int>(record.qr_value);
      }
      out << '\n';
    }
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to write inventory CSV: %s", ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::loadInventoryCsv(std::vector<InventoryRecord> & records) const
{
  std::ifstream in(inventory_file_path_);
  if (!in) {
    return false;
  }

  std::vector<InventoryRecord> loaded;
  std::string line;
  bool first_line = true;
  while (std::getline(in, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }

    const auto fields = splitCsvLine(line);
    if (fields.size() < 8 || fields[0].empty()) {
      continue;
    }

    try {
      InventoryRecord record;
      record.slot_name = fields[0];
      record.slot_code = static_cast<std::uint8_t>(std::stoul(fields[1], nullptr, 0));
      record.target = target(
        std::stod(fields[2]),
        std::stod(fields[3]),
        std::stod(fields[4]),
        std::stod(fields[5]),
        true,
        std::stoi(fields[6]) != 0,
        false,
        record.slot_name,
        record.slot_code);
      if (!fields[7].empty()) {
        const unsigned long qr = std::stoul(fields[7], nullptr, 0);
        if (qr <= 255UL) {
          record.qr_value = static_cast<std::uint8_t>(qr);
          record.has_qr_value = isValidQrValue(record.qr_value);
        }
      }
      loaded.push_back(record);
    } catch (const std::exception &) {
      continue;
    }
  }

  if (loaded.empty()) {
    return false;
  }

  records = loaded;
  return true;
}

bool RouteTargetPublisherNode::findInventoryRecordByQr(
  std::uint8_t qr_value,
  InventoryRecord & record) const
{
  std::vector<InventoryRecord> records;
  if (!loadInventoryCsv(records)) {
    records = inventory_records_;
  }

  for (const auto & candidate : records) {
    if (candidate.has_qr_value && candidate.qr_value == qr_value) {
      record = candidate;
      return true;
    }
  }
  return false;
}

bool RouteTargetPublisherNode::buildTargetRouteForQr(
  std::uint8_t qr_value,
  std::vector<Target> & route) const
{
  InventoryRecord record;
  if (!findInventoryRecordByQr(qr_value, record)) {
    return false;
  }

  route.clear();
  route.push_back(target(0.0, 0.0, 128.0, 0.0));

  const int face = slotFaceIndex(record);
  if (face == 1) {
    route.push_back(target(-10.0, -40.0, 128.0, 0.0));
    route.push_back(target(-10.0, -40.0, 128.0, 180.0, false, false, true));
    route.push_back(target(-10.0, -110.0, 128.0, 180.0, false, true, false));
  } else if (face == 2) {
    route.push_back(target(175.0, -110.0, 128.0, 0.0));
  } else if (face == 3) {
    route.push_back(target(-10.0, -245.0, 128.0, 0.0));
    route.push_back(target(-10.0, -245.0, 128.0, 180.0, false, false, true));
    route.push_back(target(-10.0, -305.0, 128.0, 180.0, false, true, false));
  }

  Target qr_target = record.target;
  qr_target.require_visual_align = true;
  route.push_back(qr_target);

  if (face == 0) {
    route.push_back(target(260.0, -40.0, 128.0, 0.0));
    route.push_back(target(260.0, -350.0, 128.0, 0.0));
    route.push_back(target(250.0, -350.0, 128.0, 0.0));
    route.push_back(target(250.0, -350.0, 0.0, 0.0));
  } else if (face == 1) {
    route.push_back(target(260.0, -110.0, 128.0, 0.0, false, true, false));
    route.push_back(target(260.0, -350.0, 128.0, 0.0, false, true, false));
    route.push_back(target(250.0, -350.0, 128.0, 0.0, false, true, false));
    route.push_back(target(250.0, -350.0, 0.0, 0.0, false, true, false));
  } else if (face == 2) {
    route.push_back(target(260.0, -245.0, 128.0, 0.0));
    route.push_back(target(260.0, -350.0, 128.0, 0.0));
    route.push_back(target(250.0, -350.0, 128.0, 0.0));
    route.push_back(target(250.0, -350.0, 0.0, 0.0));
  } else if (face == 3) {
    route.push_back(target(250.0, -350.0, 128.0, 0.0, false, true, false));
    route.push_back(target(250.0, -350.0, 0.0, 0.0, false, true, false));
  }

  RCLCPP_INFO(
    get_logger(), "Target QR %u matched %s, route size=%zu.",
    static_cast<unsigned int>(qr_value), record.slot_name.c_str(), route.size());
  return true;
}

std::vector<InventoryRecord> RouteTargetPublisherNode::makeEmptyInventoryRecords() const
{
  std::vector<InventoryRecord> records;
  records.reserve(shelfTargets().size());
  for (const auto & shelf_target : shelfTargets()) {
    InventoryRecord record;
    record.slot_name = shelf_target.slot_name;
    record.slot_code = shelf_target.slot_code;
    record.target = shelf_target;
    records.push_back(record);
  }
  return records;
}

std::string RouteTargetPublisherNode::currentQrSlotName(const Target & target) const
{
  if (!target.slot_name.empty()) {
    return target.slot_name;
  }
  return "target";
}

std::string RouteTargetPublisherNode::slotCodeToString(std::uint8_t slot_code) const
{
  std::ostringstream out;
  out << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<unsigned int>(slot_code);
  return out.str();
}

int RouteTargetPublisherNode::slotFaceIndex(const InventoryRecord & record) const
{
  if (!record.slot_name.empty()) {
    const char face = static_cast<char>(std::toupper(static_cast<unsigned char>(record.slot_name[0])));
    if (face >= 'A' && face <= 'D') {
      return face - 'A';
    }
  }

  const std::uint8_t high = static_cast<std::uint8_t>((record.slot_code & 0xF0) >> 4);
  if (high >= 0x0A && high <= 0x0D) {
    return high - 0x0A;
  }
  return 0;
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
      currentQrSlotName(target).c_str(), current_idx_);
  } else {
    publishQrTaskActive(true);
  }

  if (!has_qr_aligned_ || !qr_aligned_) {
    publishFineTuneTarget(target, x_cm, y_cm, z_cm);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for QR visual alignment at %s.", currentQrSlotName(target).c_str());
    return true;
  }

  if (!qr_code_recorded_) {
    if (!has_latest_qr_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Aligned, waiting for /qr/id.");
      return true;
    }

    std::uint8_t qr_value = 0;
    if (!parseQrCodeByte(latest_qr_id_, qr_value) || !isValidQrValue(qr_value)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid QR '%s'; expected 1-24.", latest_qr_id_.c_str());
      return true;
    }

    if (mission_state_ == MissionState::TARGET_FLIGHT && has_target_qr_value_ &&
      qr_value != target_qr_value_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target QR mismatch: expected %u got %u.",
        static_cast<unsigned int>(target_qr_value_), static_cast<unsigned int>(qr_value));
      return true;
    }

    if (mission_state_ == MissionState::INVENTORY_FLIGHT &&
      !recordInventoryValue(target, qr_value))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to record inventory value for %s.", currentQrSlotName(target).c_str());
      return true;
    }

    recorded_qr_value_ = qr_value;
    qr_code_recorded_ = true;
    RCLCPP_INFO(
      get_logger(), "QR recorded at %s: raw='%s' value=%u.",
      currentQrSlotName(target).c_str(), latest_qr_id_.c_str(),
      static_cast<unsigned int>(qr_value));
  }

  if (!laser_fire_active_) {
    laser_fire_active_ = true;
    laser_fire_start_time_ = now();
    publishLaserFire(true);
    if (mission_state_ == MissionState::INVENTORY_FLIGHT && !inventory_event_sent_) {
      publishInventoryEvent(target, recorded_qr_value_);
      inventory_event_sent_ = true;
    }
    RCLCPP_INFO(get_logger(), "Laser firing for %.1fs.", laser_fire_duration_sec_);
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

bool RouteTargetPublisherNode::publishFineTuneTarget(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm)
{
  if (!has_qr_fine_offset_) {
    return false;
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

void RouteTargetPublisherNode::advanceToNextTarget()
{
  publishQrTaskActive(false);
  publishLaserFire(false);
  qr_sequence_active_ = false;
  laser_fire_active_ = false;
  inventory_event_sent_ = false;
  fine_anchor_valid_ = false;

  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    const MissionState finished_state = mission_state_;
    mission_state_ = MissionState::COMPLETE;
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    if (finished_state == MissionState::INVENTORY_FLIGHT) {
      writeInventoryCsv();
    }
    RCLCPP_INFO(get_logger(), "Mission complete.");
  }
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (mission_state_ == MissionState::WAIT_FOR_MODE) {
    publishQrTaskActive(false);
    publishLaserFire(false);
    return;
  }

  if (mission_state_ == MissionState::TARGET_WAIT_QR) {
    publishQrTaskActive(true);
    publishLaserFire(false);

    if (!has_latest_qr_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for hand-held target QR before takeoff.");
      return;
    }

    std::uint8_t qr_value = 0;
    if (!parseQrCodeByte(latest_qr_id_, qr_value) || !isValidQrValue(qr_value)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid hand-held target QR '%s'; expected 1-24.", latest_qr_id_.c_str());
      return;
    }

    target_qr_value_ = qr_value;
    has_target_qr_value_ = true;
    target_qr_received_time_ = now();

    InventoryRecord matched_record;
    if (!findInventoryRecordByQr(target_qr_value_, matched_record)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target QR %u has no inventory match in %s. Waiting.",
        static_cast<unsigned int>(target_qr_value_), inventory_file_path_.c_str());
      has_target_qr_value_ = false;
      return;
    }

    publishTargetEvent(matched_record, target_qr_value_);
    mission_state_ = MissionState::TARGET_DELAY;
    publishQrTaskActive(false);
    latest_qr_id_.clear();
    has_latest_qr_id_ = false;
    RCLCPP_INFO(
      get_logger(), "Target QR %u read. Waiting %.1fs before takeoff.",
      static_cast<unsigned int>(target_qr_value_), target_start_delay_sec_);
    return;
  }

  if (mission_state_ == MissionState::TARGET_DELAY) {
    publishQrTaskActive(false);
    publishLaserFire(false);

    if (!has_target_qr_value_) {
      mission_state_ = MissionState::TARGET_WAIT_QR;
      return;
    }

    if ((now() - target_qr_received_time_).seconds() < target_start_delay_sec_) {
      return;
    }

    std::vector<Target> route;
    if (!buildTargetRouteForQr(target_qr_value_, route)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No inventory match for QR %u in %s. Waiting.",
        static_cast<unsigned int>(target_qr_value_), inventory_file_path_.c_str());
      return;
    }

    setRoute(route, MissionState::TARGET_FLIGHT);
    return;
  }

  if (mission_state_ == MissionState::COMPLETE) {
    publishQrTaskActive(false);
    publishLaserFire(false);
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Mission already complete.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  Target current = targets_[current_idx_];
  current.yaw_only = current.yaw_only || isYawOnlyTransition(current_idx_);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Current target %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f slot=%s",
    current_idx_ + 1, targets_.size(), current.x_cm, current.y_cm, current.z_cm,
    current.yaw_deg, current.slot_name.empty() ? "-" : current.slot_name.c_str());

  if (qr_sequence_active_) {
    handleQrTarget(current, x_cm, y_cm, z_cm);
    return;
  }

  if (isReached(current, x_cm, y_cm, z_cm, yaw_deg)) {
    if (current.require_visual_align && handleQrTarget(current, x_cm, y_cm, z_cm)) {
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
  RCLCPP_INFO(
    get_logger(),
    "RouteTestNode is passive now. Publish /mission_mode=1 or 2 to start a mission.");
}

void RouteTestNode::addTimerCallback()
{
  (void)route_node_;
}

}  // namespace activity_control_pkg
