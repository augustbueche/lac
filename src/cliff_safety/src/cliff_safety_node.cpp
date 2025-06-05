#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CliffSafetyNode : public rclcpp::Node {
public:
  CliffSafetyNode() : Node("cliff_safety_node") {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    ultra_l_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "ultrasonic_l", 10,
      std::bind(&CliffSafetyNode::cliffCallback, this, std::placeholders::_1));
    ultra_r_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "ultrasonic_r", 10,
      std::bind(&CliffSafetyNode::cliffCallback, this, std::placeholders::_1));

    cliff_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CliffSafetyNode::runSafetyLogic, this));
  }

private:
  enum class CliffState { NONE, BACKUP, ROTATE };
  CliffState state_ = CliffState::NONE;
  rclcpp::Time state_start_;

  void cliffCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    if (msg->range > cliff_threshold_ && state_ == CliffState::NONE) {
      state_ = CliffState::BACKUP;
      state_start_ = clock_->now();
      RCLCPP_WARN(this->get_logger(), "Cliff detected! Starting recovery.");
    }
  }

  void runSafetyLogic() {
    auto now = clock_->now();
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = now;
    cmd_vel.header.frame_id = "base_link";

    if (state_ == CliffState::BACKUP) {
      if ((now - state_start_).seconds() < backup_duration_) {
        cmd_vel.twist.linear.x = -backup_speed_;
        cmd_vel.twist.angular.z = 0.0;
        cliff_pub_->publish(cmd_vel);
      } else {
        state_ = CliffState::ROTATE;
        state_start_ = now;
      }
      return;
    }

    if (state_ == CliffState::ROTATE) {
      if ((now - state_start_).seconds() < rotate_duration_) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = rotate_speed_;
        cliff_pub_->publish(cmd_vel);
      } else {
        state_ = CliffState::NONE;
        RCLCPP_INFO(this->get_logger(), "Cliff recovery complete. Yielding control.");
      }
      return;
    }

    // CliffState::NONE → do nothing → let twist_mux route other commands
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_l_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultra_r_sub_;

  // Publisher and timer
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cliff_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  rclcpp::Clock::SharedPtr clock_;
  const double cliff_threshold_ = 0.15;   // m
  const double backup_duration_ = 1.0;    // seconds (20 cm at 0.2 m/s)
  const double rotate_duration_ = 1.0;    // seconds (90 deg at 1.57 rad/s)
  const double backup_speed_ = 0.2;       // m/s
  const double rotate_speed_ = 1.57;      // rad/s (90°/s)
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CliffSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
