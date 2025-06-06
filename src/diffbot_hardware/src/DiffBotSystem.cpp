#include "diffbot_hardware/DiffBotSystem.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sstream>
#include <vector>
#include <optional>
#include <cmath>   // <-- for M_PI

namespace diffbot_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

hardware_interface::CallbackReturn DiffBotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto & hw_params = info.hardware_parameters;
  if (!hw_params.count("serial_port") || !hw_params.count("baud_rate")) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystem"),
                 "Missing 'serial_port' or 'baud_rate' in hardware_parameters");
    return CallbackReturn::ERROR;
  }
  port_      = hw_params.at("serial_port");
  baud_rate_ = std::stod(hw_params.at("baud_rate"));

  // initialize commands and states
  left_cmd_   = right_cmd_   = 0.0;
  left_vel_   = right_vel_   = 0.0;
  left_pos_   = right_pos_   = 0.0;
  prev_left_  = prev_right_  = 0.0;

  try {
    serial_port_.setPort(port_);
    serial_port_.setBaudrate(static_cast<uint32_t>(baud_rate_));
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    serial_port_.setTimeout(to);
    serial_port_.open();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystem"),
      "Failed to open serial port '%s': %s", port_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  // ROS 2 node handle and ultrasonic publishers
  sensor_node_ = rclcpp::Node::make_shared("ultrasonic_sensor_node");
  ultra_pub_a_ = sensor_node_->create_publisher<sensor_msgs::msg::Range>("ultrasonic_a", 10);
  ultra_pub_b_ = sensor_node_->create_publisher<sensor_msgs::msg::Range>("ultrasonic_b", 10);
  ultra_pub_c_ = sensor_node_->create_publisher<sensor_msgs::msg::Range>("ultrasonic_c", 10);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_pos_);
  interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_vel_);
  interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_pos_);
  interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_vel_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_cmd_);
  interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_cmd_);
  return interfaces;
}

hardware_interface::return_type DiffBotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  while (serial_port_.available()) {
    std::string line = serial_port_.readline(256, "\n");

    if (line.find("Left Encoder:") != std::string::npos) {
      auto parse_val = [&](const std::string & key) -> std::optional<double> {
        auto pos = line.find(key);
        if (pos == std::string::npos) return std::nullopt;
        return std::stod(line.substr(pos + key.size()));
      };

      auto l_opt = parse_val("Left Encoder:");
      auto r_opt = parse_val("Right Encoder:");

      if (l_opt && r_opt) {
        double curr_left  = *l_opt;
        double curr_right = *r_opt;

        // ─── encoder conversion ────────────────────────────────
        //  count edges on channel A → 32 CPR; multiply by gearbox ratio
        constexpr double encoder_cpr    = 64.0;
        constexpr double gearbox_ratio  = 102.08;       // adjust to your exact gearbox!
        constexpr double counts_per_rev = encoder_cpr * gearbox_ratio;
        constexpr double rad_per_count  = 2.0 * M_PI / counts_per_rev;

        double dt = period.seconds();
        if (dt > 0.0) {
          double d_left  = (curr_left  - prev_left_)  * rad_per_count;
          double d_right = (curr_right - prev_right_) * rad_per_count;
          left_vel_  = d_left  / dt;  // rad/s
          right_vel_ = d_right / dt;  // rad/s
        }

        left_pos_  = curr_left  * rad_per_count;  // rad
        right_pos_ = curr_right * rad_per_count;  // rad

        prev_left_  = curr_left;
        prev_right_ = curr_right;
        // ────────────────────────────────────────────────────────

      }
    }

    if (line.find("UltraA:") != std::string::npos) {
      auto parse_val = [&](const std::string & key) -> std::optional<double> {
        auto pos = line.find(key);
        if (pos == std::string::npos) return std::nullopt;
        return std::stod(line.substr(pos + key.size()));
      };

      auto a = parse_val("UltraA:");
      auto b = parse_val("UltraB:");
      auto c = parse_val("UltraC:");

      auto publish_range = [&](std::optional<double> dist_cm, auto pub, const std::string & frame_id) {
        if (!dist_cm) return;
        sensor_msgs::msg::Range msg;
        msg.header.stamp = sensor_node_->get_clock()->now();
        msg.header.frame_id = frame_id;
        msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        msg.field_of_view = 0.3;
        msg.min_range = 0.02;
        msg.max_range = 4.0;
        msg.range = *dist_cm / 100.0;  // cm → m
        pub->publish(msg);
      };

      publish_range(a, ultra_pub_a_, "ultrasonic_a_link");
      publish_range(b, ultra_pub_b_, "ultrasonic_b_link");
      publish_range(c, ultra_pub_c_, "ultrasonic_c_link");
    }
  }

  return return_type::OK;
}

hardware_interface::return_type DiffBotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_port_.isOpen()) {
    int left_pwm  = static_cast<int>(left_cmd_ * 100.0);
    int right_pwm = static_cast<int>(right_cmd_ * 100.0);

    std::ostringstream oss;
    oss << left_pwm << "," << right_pwm << "\n";
    serial_port_.write(oss.str());
  }
  return return_type::OK;
}

}  // namespace diffbot_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::DiffBotSystem,
  hardware_interface::SystemInterface
)
