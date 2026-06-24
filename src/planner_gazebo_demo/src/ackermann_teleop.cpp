// ackermann_teleop.cpp
//
// Keyboard teleop for the Ackermann car — drives like a REAL car.
//
// The ros2_controllers AckermannSteeringController subscribes to a plain
// geometry_msgs/msg/Twist on ~/reference_unstamped, interpreting twist.linear.x
// as forward speed [m/s] and twist.angular.z as YAW RATE [rad/s], and internally
// converting (v, w) -> rear-wheel speed + front steering angle delta = atan(L*w/v).
//
// KEY POINT: we let the keys control the STEERING ANGLE delta (like a steering
// wheel), NOT the yaw rate directly. We then publish w = v*tan(delta)/L so the
// controller reproduces exactly that steering angle. This makes the turn radius
// (= L/tan(delta)) depend only on the steering angle, independent of speed — the
// intuitive, car-like behaviour. (Mapping keys to yaw rate, as before, made the
// steering blow up at low speed and felt like the car was spinning oddly.)
//
// Only FRONT wheels steer, only REAR wheels drive — that is standard Ackermann.
//
// Controls:  w/s = throttle (speed +/-)   a/d = steer left/right (held)
//            space = stop & centre         q = quit

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
    wheelbase_  = this->declare_parameter("wheelbase", 0.9);
    steer_max_  = this->declare_parameter("steer_max", 0.6109);   // 35 deg
    speed_step_ = this->declare_parameter("speed_step", 0.1);     // m/s per key
    steer_step_ = this->declare_parameter("steer_step", 0.087);   // ~5 deg per key
    speed_max_  = this->declare_parameter("speed_max", 1.2);      // m/s (gentle for mapping)

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_steering_controller/reference_unstamped", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AckermannTeleop::timerCallback, this));
    RCLCPP_INFO(this->get_logger(),
      "Teleop (car-like): w/s=throttle, a/d=steer, space=stop&centre, q=quit. "
      "a/d set the STEERING ANGLE (turn radius fixed by angle, not speed).");
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
          case 'w': speed_ += speed_step_; break;
          case 's': speed_ -= speed_step_; break;
          case 'a': steer_ += steer_step_; break;   // steering ANGLE, left +
          case 'd': steer_ -= steer_step_; break;
          case ' ': speed_ = 0.0; steer_ = 0.0; break;
          case 'q': rclcpp::shutdown(); break;
        }
        speed_ = std::clamp(speed_, -speed_max_, speed_max_);
        steer_ = std::clamp(steer_, -steer_max_, steer_max_);
        RCLCPP_INFO(this->get_logger(), "speed=%.2f m/s  steer=%.1f deg",
          speed_, steer_ * 180.0 / M_PI);
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
    // yaw rate that yields steering angle `steer_` at the current speed:
    //   delta = atan(L*w/v)  ->  w = v*tan(delta)/L
    msg.angular.z = speed_ * std::tan(steer_) / wheelbase_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double speed_ = 0.0;   // m/s
  double steer_ = 0.0;   // rad (steering angle, held)
  double wheelbase_, steer_max_, speed_step_, steer_step_, speed_max_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannTeleop>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
