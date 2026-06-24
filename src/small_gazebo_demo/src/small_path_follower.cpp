#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SmallPathFollower : public rclcpp::Node
{
public:
  SmallPathFollower()
  : Node("small_path_follower"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cruise_speed_ = declare_parameter("cruise_speed", 0.18);
    max_angular_ = declare_parameter("max_angular", 0.85);
    lookahead_ = declare_parameter("lookahead", 0.65);
    goal_tol_ = declare_parameter("goal_tol", 0.22);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planner/path", rclcpp::QoS(10),
      std::bind(&SmallPathFollower::pathCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&SmallPathFollower::tick, this));

    RCLCPP_INFO(get_logger(), "small_path_follower waiting for /planner/path.");
  }

private:
  struct Point2
  {
    double x;
    double y;
  };

  static double wrap(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<Point2> pts;
    pts.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      pts.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    if (pts.size() >= 2) {
      path_ = std::move(pts);
      reached_ = false;
      RCLCPP_INFO(get_logger(), "received planner path with %zu points.", path_.size());
    }
  }

  bool lookupPose(double & x, double & y, double & yaw)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void tick()
  {
    geometry_msgs::msg::Twist cmd;
    if (path_.size() < 2 || reached_) {
      cmd_pub_->publish(cmd);
      return;
    }

    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!lookupPose(x, y, yaw)) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double goal_dist = std::hypot(path_.back().x - x, path_.back().y - y);
    if (goal_dist < goal_tol_) {
      reached_ = true;
      RCLCPP_INFO(get_logger(), "reached planner goal; stopping.");
      cmd_pub_->publish(cmd);
      return;
    }

    size_t nearest = 0;
    double best = 1e9;
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = std::hypot(path_[i].x - x, path_[i].y - y);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }

    size_t target = nearest;
    double travelled = 0.0;
    while (target + 1 < path_.size() && travelled < lookahead_) {
      travelled += std::hypot(
        path_[target + 1].x - path_[target].x,
        path_[target + 1].y - path_[target].y);
      ++target;
    }

    const double target_yaw = std::atan2(path_[target].y - y, path_[target].x - x);
    const double alpha = wrap(target_yaw - yaw);
    const double ld = std::max(std::hypot(path_[target].x - x, path_[target].y - y), 0.25);

    cmd.angular.z = std::clamp(2.0 * cruise_speed_ * std::sin(alpha) / ld,
                               -max_angular_, max_angular_);
    const double heading_scale = std::clamp(1.0 - std::abs(alpha) / 1.4, 0.20, 1.0);
    cmd.linear.x = cruise_speed_ * heading_scale;
    cmd_pub_->publish(cmd);

    if (++log_count_ % 40 == 0) {
      RCLCPP_INFO(get_logger(), "pose=(%.2f, %.2f) goal_dist=%.2f", x, y, goal_dist);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<Point2> path_;
  bool reached_ = false;
  int log_count_ = 0;
  double cruise_speed_ = 0.18;
  double max_angular_ = 0.85;
  double lookahead_ = 0.65;
  double goal_tol_ = 0.22;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmallPathFollower>());
  rclcpp::shutdown();
  return 0;
}
