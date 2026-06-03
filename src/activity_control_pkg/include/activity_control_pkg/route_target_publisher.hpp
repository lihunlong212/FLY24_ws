#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
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
  bool require_visual_align = false;
  bool invert_xy_velocity = false;
  bool yaw_only = false;
  std::string slot_name{};
  std::uint8_t slot_code = 0;
};

enum class MissionState : std::uint8_t
{
  WAIT_FOR_MODE = 0,
  INVENTORY_FLIGHT = 1,
  TARGET_WAIT_QR = 2,
  TARGET_DELAY = 3,
  TARGET_FLIGHT = 4,
  COMPLETE = 5
};

struct InventoryRecord
{
  std::string slot_name;
  std::uint8_t slot_code = 0;
  Target target;
  bool has_qr_value = false;
  std::uint8_t qr_value = 0;
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  std::size_t currentIndex() const;
  std::size_t size() const;

private:
  void missionModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void startInventoryMission();
  void startTargetMissionPreflight();
  void setRoute(const std::vector<Target> & route, MissionState state);

  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);
  void publishQrTaskActive(bool active);
  void publishLaserFire(bool fire);
  void publishInventoryData();
  void publishInventoryEvent(const Target & target, std::uint8_t qr_value);
  void publishTargetEvent(const InventoryRecord & record, std::uint8_t qr_value);

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg)
    const;
  bool isYawOnlyTransition(std::size_t index) const;
  bool handleQrTarget(const Target & target, double x_cm, double y_cm, double z_cm);
  bool publishFineTuneTarget(const Target & target, double x_cm, double y_cm, double z_cm);
  void advanceToNextTarget();
  void resetQrStateForTarget();

  bool parseQrCodeByte(const std::string & text, std::uint8_t & value) const;
  bool isValidQrValue(std::uint8_t value) const;
  bool recordInventoryValue(const Target & target, std::uint8_t qr_value);
  bool writeInventoryCsv();
  bool loadInventoryCsv(std::vector<InventoryRecord> & records) const;
  bool findInventoryRecordByQr(std::uint8_t qr_value, InventoryRecord & record) const;
  bool buildTargetRouteForQr(std::uint8_t qr_value, std::vector<Target> & route) const;

  std::vector<InventoryRecord> makeEmptyInventoryRecords() const;
  std::string currentQrSlotName(const Target & target) const;
  std::string slotCodeToString(std::uint8_t slot_code) const;
  int slotFaceIndex(const InventoryRecord & record) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void qrIdCallback(const std_msgs::msg::String::SharedPtr msg);
  void qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void qrFineOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg);

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr qr_task_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr qr_laser_fire_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr qr_inventory_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr qr_inventory_event_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr qr_target_event_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mission_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_aligned_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_fine_offset_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::vector<InventoryRecord> inventory_records_;
  std::size_t current_idx_;
  MissionState mission_state_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;
  std::string inventory_file_path_;

  std::string latest_qr_id_;
  bool has_latest_qr_id_{false};
  bool qr_aligned_{false};
  bool has_qr_aligned_{false};
  geometry_msgs::msg::Point qr_fine_offset_body_cm_{};
  bool has_qr_fine_offset_{false};

  bool qr_sequence_active_{false};
  bool qr_code_recorded_{false};
  bool laser_fire_active_{false};
  bool inventory_event_sent_{false};
  std::uint8_t recorded_qr_value_{0};
  rclcpp::Time laser_fire_start_time_;
  double laser_fire_duration_sec_{1.0};

  bool has_target_qr_value_{false};
  std::uint8_t target_qr_value_{0};
  rclcpp::Time target_qr_received_time_;
  double target_start_delay_sec_{5.0};

  double fine_offset_limit_cm_{15.0};
  double fine_target_publish_hz_{5.0};

  bool fine_anchor_valid_{false};
  std::size_t fine_anchor_target_idx_{0};
  double fine_anchor_x_cm_{0.0};
  double fine_anchor_y_cm_{0.0};
  double fine_anchor_z_cm_{0.0};
  rclcpp::Time fine_last_publish_time_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void addTimerCallback();

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::TimerBase::SharedPtr add_timer_;

  bool started_;
  std::size_t next_target_index_;
};

}  // namespace activity_control_pkg
