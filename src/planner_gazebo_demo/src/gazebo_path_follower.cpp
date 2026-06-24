// gazebo_path_follower.cpp
//
// Closes the loop: drives the REAL Gazebo car along the planned path.
//   - subscribes to /planner/path (in the map frame),
//   - reads the car's pose from TF (map -> base_link),
//   - pure-pursuit: picks a look-ahead point on the path and computes the
//     Ackermann steering toward it,
//   - publishes geometry_msgs/Twist to the controller's reference_unstamped topic
//     (linear.x = slow cruise speed, angular.z = yaw rate),
//   - stops when the car is within goal_tol of the path end.
//
// This is the "planning -> control execution" half of perception->planning->control:
// the path computed on the map is actually executed by the physics-simulated car.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class GazeboPathFollower : public rclcpp::Node
{
public:
  GazeboPathFollower()
  : Node("gazebo_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cruise_     = this->declare_parameter("cruise_speed", 0.5);   // m/s — slow
    lookahead_  = this->declare_parameter("lookahead", 1.5);      // m
    wheelbase_  = this->declare_parameter("wheelbase", 0.9);
    steer_max_  = this->declare_parameter("steer_max", 0.6109);
    goal_tol_   = this->declare_parameter("goal_tol", 0.6);       // m
    map_frame_  = this->declare_parameter("map_frame", std::string("map"));
    base_frame_ = this->declare_parameter("base_frame", std::string("base_link"));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_steering_controller/reference_unstamped", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planner/path", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::SharedPtr m) {
        std::vector<P> pts;
        for (const auto & ps : m->poses) {pts.push_back({ps.pose.position.x, ps.pose.position.y});}
        if (pts.size() >= 2) {pts_ = pts;}
      });
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GazeboPathFollower::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "gazebo_path_follower: driving the car along /planner/path at %.2f m/s.", cruise_);
  }

private:
  struct P {double x; double y;};
  static double wrap(double a) {while (a > M_PI) {a -= 2*M_PI;} while (a < -M_PI) {a += 2*M_PI;} return a;}

  void tick()
  {
    geometry_msgs::msg::Twist cmd;   // default zero = stop
    if (pts_.size() < 2 || reached_) {pub_->publish(cmd); return;}

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception &) {
      pub_->publish(cmd); return;     // no localization yet
    }
    const double cx = tf.transform.translation.x;
    const double cy = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (std::hypot(pts_.back().x - cx, pts_.back().y - cy) < goal_tol_) {
      reached_ = true;
      RCLCPP_INFO(this->get_logger(), "reached goal (%.2f, %.2f) — stopping.",
        pts_.back().x, pts_.back().y);
      pub_->publish(cmd); return;
    }

    // nearest path point, then walk forward by the look-ahead distance
    size_t ni = 0; double bd = 1e9;
    for (size_t i = 0; i < pts_.size(); ++i) {
      const double d = std::hypot(pts_[i].x - cx, pts_[i].y - cy);
      if (d < bd) {bd = d; ni = i;}
    }
    size_t ti = ni; double acc = 0.0;
    while (ti + 1 < pts_.size() && acc < lookahead_) {
      acc += std::hypot(pts_[ti+1].x - pts_[ti].x, pts_[ti+1].y - pts_[ti].y);
      ++ti;
    }
    const double tx = pts_[ti].x, ty = pts_[ti].y;

    // Ackermann pure pursuit
    const double alpha = wrap(std::atan2(ty - cy, tx - cx) - yaw);
    double steer;
    if (std::abs(alpha) > 1.6) {
      steer = (alpha > 0.0) ? steer_max_ : -steer_max_;
    } else {
      const double Ld = std::max(std::hypot(tx - cx, ty - cy), 0.5);
      steer = std::clamp(std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld),
                         -steer_max_, steer_max_);
    }
    cmd.linear.x = cruise_;
    cmd.angular.z = cruise_ * std::tan(steer) / wheelbase_;
    pub_->publish(cmd);

    if (++log_ % 40 == 0) {
      RCLCPP_INFO(this->get_logger(), "car=(%.2f,%.2f) -> goal dist=%.2f m",
        cx, cy, std::hypot(pts_.back().x - cx, pts_.back().y - cy));
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<P> pts_;
  bool reached_ = false;
  int log_ = 0;
  double cruise_, lookahead_, wheelbase_, steer_max_, goal_tol_;
  std::string map_frame_, base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPathFollower>());
  rclcpp::shutdown();
  return 0;
}
