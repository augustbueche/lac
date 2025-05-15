#ifndef DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <string>
#include <vector>
#include <memory>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <serial/serial.h>

namespace diffbot_hardware
{

class DiffBotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial Communication
  serial::Serial serial_port_;
  std::string port_;
  double baud_rate_;

  // Wheel feedback anc commands
  double left_vel_{0.0}, right_vel_{0.0};
  double left_cmd_{0.0}, right_cmd_{0.0};
  double left_pos_{0.0}, right_pos_{0.0};
  double wheel_radius_{0.0325};  // meters, from your Xacro

  // ROS pulishers for ultrasonic sensors
  rclcpp::Node::SharedPtr sensor_node_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultra_pub_a_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultra_pub_b_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultra_pub_c_;

  // Cached distance readings (this is storing the ultrasonic sensor readings)
  double ultra_a_{-1.0};
  double ultra_b_{-1.0};
  double ultra_c_{-1.0};
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
