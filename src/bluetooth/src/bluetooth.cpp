#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

class BluetoothNode : public rclcpp::Node
{
public:
  BluetoothNode()
  : Node("bluetooth_node")
  {
    declare_parameter<std::string>("port", "/dev/ttyS3");
    declare_parameter<int>("baudrate", 9600);

    const std::string port = get_parameter("port").as_string();
    const int baudrate = get_parameter("baudrate").as_int();

    inventory_event_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/qr/inventory_event", rclcpp::QoS(10),
      std::bind(&BluetoothNode::inventoryEventCallback, this, std::placeholders::_1));
    target_event_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/qr/target_event", rclcpp::QoS(10),
      std::bind(&BluetoothNode::targetEventCallback, this, std::placeholders::_1));

    if (openAndConfigureSerialPort(port, baudrate)) {
      RCLCPP_INFO(get_logger(), "Bluetooth inventory sender ready on %s.", port.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Bluetooth serial unavailable; inventory frames will be dropped.");
    }
  }

  ~BluetoothNode() override
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  static speed_t baudToConstant(int baudrate)
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      default:
        return B9600;
    }
  }

  bool openAndConfigureSerialPort(const std::string & port, int baudrate)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port.c_str(), strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baudToConstant(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    return true;
  }

  void inventoryEventCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "Inventory event requires [slot_code, qr_value].");
      return;
    }

    const std::uint8_t slot_code = msg->data[0];
    const std::uint8_t qr_value = msg->data[1];
    const std::vector<std::uint8_t> frame = {0xAA, slot_code, qr_value, 0xFF};
    sendFrame(frame, "inventory", slot_code, qr_value);
  }

  void targetEventCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "Target event requires [slot_code, qr_value].");
      return;
    }

    const std::uint8_t slot_code = msg->data[0];
    const std::uint8_t qr_value = msg->data[1];
    const std::vector<std::uint8_t> frame = {0xAA, slot_code, qr_value, 0xFF, 0xFF};
    sendFrame(frame, "target", slot_code, qr_value);
  }

  void sendFrame(
    const std::vector<std::uint8_t> & frame,
    const char * label,
    std::uint8_t slot_code,
    std::uint8_t qr_value)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Bluetooth serial not open; dropped %s frame slot=0x%02X qr=0x%02X.",
        label, static_cast<unsigned int>(slot_code), static_cast<unsigned int>(qr_value));
      return;
    }

    const ssize_t written = write(serial_fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_ERROR(get_logger(), "Failed to send complete Bluetooth inventory frame: %s", strerror(errno));
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Sent Bluetooth %s frame slot=0x%02X qr=0x%02X len=%zu.",
      label, static_cast<unsigned int>(slot_code), static_cast<unsigned int>(qr_value),
      frame.size());
  }

  int serial_fd_{-1};
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inventory_event_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr target_event_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BluetoothNode>());
  rclcpp::shutdown();
  return 0;
}
