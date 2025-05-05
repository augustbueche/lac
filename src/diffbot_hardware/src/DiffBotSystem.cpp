#include "diffbot_hardware/DiffBotSystem.hpp"


#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>


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


 try {
   serial_port_.setPort(port_);
   serial_port_.setBaudrate(static_cast<uint32_t>(baud_rate_));
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);
   serial_port_.setTimeout(to);
   serial_port_.open();
 } catch (const std::exception & e) {
   RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystem"),
     "Failed to open serial port '%s': %s", port_.c_str(), e.what());
   return CallbackReturn::ERROR;
 }


 return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
DiffBotSystem::export_state_interfaces()
{
 std::vector<hardware_interface::StateInterface> interfaces;
 interfaces.emplace_back(
   info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_vel_);
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
 const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
 if (serial_port_.available()) {
   std::string line = serial_port_.readline(256, "\n");
   auto parse_val = [&](const std::string & key) {
     auto pos = line.find(key);
     if (pos == std::string::npos) return 0.0;
     return std::stod(line.substr(pos + key.size()));
   };


   double left_count  = parse_val("Left Encoder:");
   double right_count = parse_val("Right Encoder:");


   // TODO: convert counts → angular velocity
   left_vel_  = left_count;
   right_vel_ = right_count;
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


// register your SystemInterface implementation with pluginlib
PLUGINLIB_EXPORT_CLASS(
 diffbot_hardware::DiffBotSystem,
 hardware_interface::SystemInterface
)





