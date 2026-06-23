// ackermann_teleop.cpp
//
// Keyboard teleop for the Ackermann car.
//
// IMPORTANT: the ros2_controllers AckermannSteeringController subscribes to a
// plain geometry_msgs/msg/Twist on `~/reference_unstamped` (verified at runtime:
// /ackermann_steering_controller/reference has NO subscriber, while
// /ackermann_steering_controller/reference_unstamped has the controller as
// subscriber). It interprets twist.linear.x as forward speed [m/s] and
// twist.angular.z as yaw rate [rad/s], and internally converts (v, w) to the
// rear-wheel speed and Ackermann steering angle. So we publish Twist there.
//
// Controls: w/s = speed +/-, a/d = turn left/right (yaw rate), space = stop, q = quit.

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class AckermannTeleop : public rclcpp::Node
{
public:
  AckermannTeleop() : Node("ackermann_teleop")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_steering_controller/reference_unstamped", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AckermannTeleop::timerCallback, this));
    RCLCPP_INFO(this->get_logger(),
      "Teleop ready: w/s=speed, a/d=turn, space=stop, q=quit "
      "(publishing Twist -> /ackermann_steering_controller/reference_unstamped)");
  }

  void spin()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    while (rclcpp::ok()) {
      if (read(STDIN_FILENO, &c, 1) > 0) {
        switch (c) {
          case 'w': speed_ += 0.2; break;
          case 's': speed_ -= 0.2; break;
          case 'a': yaw_rate_ += 0.1; break;
          case 'd': yaw_rate_ -= 0.1; break;
          case ' ': speed_ = 0.0; yaw_rate_ = 0.0; break;
          case 'q': rclcpp::shutdown(); break;
        }
        speed_ = std::clamp(speed_, -2.0, 2.0);
        yaw_rate_ = std::clamp(yaw_rate_, -1.5, 1.5);
      }
      rclcpp::spin_some(shared_from_this());
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = speed_;
    msg.angular.z = yaw_rate_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double speed_ = 0.0;
  double yaw_rate_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannTeleop>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
