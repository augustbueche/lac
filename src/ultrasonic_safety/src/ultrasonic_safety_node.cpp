#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class UltrasonicSafetyNode : public rclcpp::Node {
public:
 UltrasonicSafetyNode() : Node("ultrasonic_safety_node") {
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);  // <--- THIS FIXES THE ERROR

  ultra_a_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "ultrasonic_a", 10,
    std::bind(&UltrasonicSafetyNode::ultrasonicCallbackA, this, std::placeholders::_1));

  ultra_b_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "ultrasonic_b", 10,
    std::bind(&UltrasonicSafetyNode::ultrasonicCallbackB, this, std::placeholders::_1));

  ultra_c_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "ultrasonic_c", 10,
    std::bind(&UltrasonicSafetyNode::ultrasonicCallbackC, this, std::placeholders::_1));

  safety_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&UltrasonicSafetyNode::publishSafetyCommand, this));
}


private:
  void ultrasonicCallbackA(const sensor_msgs::msg::Range::SharedPtr msg) {
    obstacle_a_detected_ = msg->range < safety_distance_threshold_;
  }

  void ultrasonicCallbackB(const sensor_msgs::msg::Range::SharedPtr msg) {
    obstacle_b_detected_ = msg->range < safety_distance_threshold_;
  }

  void ultrasonicCallbackC(const sensor_msgs::msg::Range::SharedPtr msg) {
    obstacle_c_detected_ = msg->range < safety_distance_threshold_;
  }

  void publishSafetyCommand() {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = "base_link";
    bool obstacle_present = obstacle_a_detected_ || obstacle_b_detected_ || obstacle_c_detected_;

    if (obstacle_present) {
      // Keep reversing and rotating while any obstacle is detected
      cmd_vel.twist.linear.x = 0.0;     // reverse
      //cmd_vel.twist.linear.x = -0.1;     // reverse
      cmd_vel.twist.angular.z = 0.5;     // rotate
      safety_pub_->publish(cmd_vel);
      last_obstacle_time_ = clock_->now();
    } else {
      // Only publish stop if we just recently exited obstacle state
      if ((clock_->now()- last_obstacle_time_).seconds() < stop_duration_) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        safety_pub_->publish(cmd_vel);
      }
      // Otherwise, don't publish anything so twist_mux can pass through other commands
    }
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_a_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_b_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_c_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Obstacle detection flags
  bool obstacle_a_detected_ = false;
  bool obstacle_b_detected_ = false;
  bool obstacle_c_detected_ = false;

  // Time tracking
  rclcpp::Time last_obstacle_time_;
  rclcpp::Clock::SharedPtr clock_;

  // Constants
  const double safety_distance_threshold_ = 0.3;  // meters
  const double stop_duration_ = 0.3;               // seconds to stop before yielding control
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSafetyNode>());
  rclcpp::shutdown();
  return 0;
}

