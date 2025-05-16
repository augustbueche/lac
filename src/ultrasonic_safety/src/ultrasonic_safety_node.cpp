#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class UltrasonicSafetyNode : public rclcpp::Node {
public:
  UltrasonicSafetyNode() : Node("ultrasonic_safety_node") {
    ultra_a_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "ultrasonic_a", 10,
        std::bind(&UltrasonicSafetyNode::ultrasonicCallback, this, std::placeholders::_1));

    ultra_b_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "ultrasonic_b", 10,
        std::bind(&UltrasonicSafetyNode::ultrasonicCallback, this, std::placeholders::_1));

    ultra_c_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "ultrasonic_c", 10,
        std::bind(&UltrasonicSafetyNode::ultrasonicCallback, this, std::placeholders::_1));

    safety_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_safety", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&UltrasonicSafetyNode::publishSafetyCommand, this));
  }

private:
  void ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    if (msg->range < safety_distance_threshold_) {
      obstacle_detected_ = true;
      last_detection_time_ = now();
    }
  }

  void publishSafetyCommand() {
    geometry_msgs::msg::TwistStamped cmd_vel;

    if (obstacle_detected_ && (now() - last_detection_time_).seconds() < obstacle_clear_time_) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      safety_pub_->publish(cmd_vel);
    }

    obstacle_detected_ = false;
  }

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_a_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_b_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_c_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool obstacle_detected_ = false;
  rclcpp::Time last_detection_time_;

  const double safety_distance_threshold_ = 0.25; // meters (25 cm)
  const double obstacle_clear_time_ = 0.5;        // seconds
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
