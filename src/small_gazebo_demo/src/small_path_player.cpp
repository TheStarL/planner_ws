#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class SmallPathPlayer : public rclcpp::Node
{
public:
  SmallPathPlayer()
  : Node("small_path_player")
  {
    play_speed_ = declare_parameter("play_speed", 0.22);
    wheel_radius_ = declare_parameter("wheel_radius", 0.07);
    frame_id_ = declare_parameter("frame_id", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    start_x_ = declare_parameter("start_x", -2.35);
    start_y_ = declare_parameter("start_y", -2.35);
    start_yaw_ = declare_parameter("start_yaw", 0.0);

    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planner/path", rclcpp::QoS(10),
      std::bind(&SmallPathPlayer::pathCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(40), std::bind(&SmallPathPlayer::tick, this));

    hold_x_ = start_x_;
    hold_y_ = start_y_;
    hold_yaw_ = start_yaw_;
    publishPose(start_x_, start_y_, start_yaw_);
    RCLCPP_INFO(get_logger(), "small_path_player waiting for /planner/path.");
  }

private:
  struct Point2
  {
    double x;
    double y;
  };

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<Point2> pts;
    pts.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      pts.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    if (pts.size() < 2) return;

    // The planner republishes the same path a few times per second. Restarting
    // the animation on every message resets the robot to the start ~5x/sec, so
    // it jitters at the start and never visibly follows the route. Only (re)start
    // playback when the path actually changes (i.e. a genuine replan).
    if (sameAsCurrentPath(pts)) return;

    path_ = std::move(pts);
    segment_ = 0;
    segment_s_ = 0.0;
    total_s_ = 0.0;
    wheel_angle_ = 0.0;
    playing_ = true;
    RCLCPP_INFO(get_logger(), "playing planner path with %zu points.", path_.size());
  }

  bool sameAsCurrentPath(const std::vector<Point2> & pts) const
  {
    if (pts.size() != path_.size()) return false;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (std::abs(pts[i].x - path_[i].x) > 1e-6 ||
          std::abs(pts[i].y - path_[i].y) > 1e-6) {
        return false;
      }
    }
    return true;
  }

  void publishPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = std::sin(0.5 * yaw);
    tf.transform.rotation.w = std::cos(0.5 * yaw);
    tf_pub_->sendTransform(tf);

    sensor_msgs::msg::JointState js;
    js.header.stamp = tf.header.stamp;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {wheel_angle_, wheel_angle_};
    joint_pub_->publish(js);
  }

  void tick()
  {
    if (!playing_ || path_.size() < 2) {
      // Idle: hold the start pose before the first path, and the goal pose after
      // finishing. Publishing the start pose here would snap the robot back to
      // the start the instant it reaches the goal.
      publishPose(hold_x_, hold_y_, hold_yaw_);
      return;
    }

    double advance = play_speed_ * 0.04;
    while (segment_ + 1 < path_.size() && advance > 0.0) {
      const auto & a = path_[segment_];
      const auto & b = path_[segment_ + 1];
      const double len = std::hypot(b.x - a.x, b.y - a.y);
      if (len < 1e-6) {
        ++segment_;
        segment_s_ = 0.0;
        continue;
      }
      const double remain = len - segment_s_;
      if (advance < remain) {
        segment_s_ += advance;
        total_s_ += advance;
        advance = 0.0;
      } else {
        advance -= remain;
        total_s_ += remain;
        ++segment_;
        segment_s_ = 0.0;
      }
    }

    if (segment_ + 1 >= path_.size()) {
      playing_ = false;
      const auto & p = path_.back();
      hold_x_ = p.x;
      hold_y_ = p.y;
      hold_yaw_ = last_yaw_;
      publishPose(p.x, p.y, last_yaw_);
      RCLCPP_INFO(get_logger(), "planner path animation reached the goal.");
      return;
    }

    const auto & a = path_[segment_];
    const auto & b = path_[segment_ + 1];
    const double len = std::max(std::hypot(b.x - a.x, b.y - a.y), 1e-6);
    const double u = std::clamp(segment_s_ / len, 0.0, 1.0);
    const double x = a.x + (b.x - a.x) * u;
    const double y = a.y + (b.y - a.y) * u;
    last_yaw_ = std::atan2(b.y - a.y, b.x - a.x);
    wheel_angle_ = total_s_ / std::max(wheel_radius_, 1e-3);
    publishPose(x, y, last_yaw_);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  std::vector<Point2> path_;
  bool playing_ = false;
  size_t segment_ = 0;
  double segment_s_ = 0.0;
  double total_s_ = 0.0;
  double wheel_angle_ = 0.0;
  double last_yaw_ = 0.0;
  double hold_x_ = 0.0;
  double hold_y_ = 0.0;
  double hold_yaw_ = 0.0;
  double play_speed_ = 0.22;
  double wheel_radius_ = 0.07;
  double start_x_ = -2.35;
  double start_y_ = -2.35;
  double start_yaw_ = 0.0;
  std::string frame_id_ = "map";
  std::string base_frame_ = "base_link";
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmallPathPlayer>());
  rclcpp::shutdown();
  return 0;
}
