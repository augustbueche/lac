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
  serial::Serial serial_port_;
  std::string port_;
  double baud_rate_;

  double left_vel_{0.0}, right_vel_{0.0};
  double left_cmd_{0.0}, right_cmd_{0.0};

  double wheel_radius_{0.0325};  // meters, from your Xacro
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
