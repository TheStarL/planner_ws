#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SmallAutoMapper : public rclcpp::Node
{
public:
  SmallAutoMapper()
  : Node("small_auto_mapper")
  {
    linear_speed_ = declare_parameter("linear_speed", 0.12);
    max_angular_ = declare_parameter("max_angular", 0.38);
    waypoint_tol_ = declare_parameter("waypoint_tolerance", 0.18);
    front_stop_ = declare_parameter("front_stop", 0.42);
    front_sector_rad_ = declare_parameter("front_sector_rad", 0.45);

    // World/odom-frame waypoints. The diff-drive plugin is configured with
    // world odometry, so odom coordinates match the small_room.world map.
    // The policy below stops to turn before translating; this avoids the scan
    // shearing that created duplicated walls during corners.
    waypoints_ = {
      {2.35, -2.35}, {2.35, 2.35}, {-2.35, 2.35}, {-2.35, -0.65},
      {1.90, -0.65}, {1.90, 1.90}, {-1.90, 1.90}, {-1.90, 0.25},
      {2.15, 0.25}, {2.15, -2.05}, {-2.10, -2.05}, {-2.10, 2.10}
    };

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SmallAutoMapper::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&SmallAutoMapper::scanCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&SmallAutoMapper::tick, this));

    RCLCPP_INFO(get_logger(), "small_auto_mapper started: slow waypoint sweep for SLAM.");
  }

private:
  static double wrap(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double yawFromOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = yawFromOdom(*msg);
    have_odom_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_front = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (std::abs(angle) > front_sector_rad_) continue;
      const double r = msg->ranges[i];
      if (std::isfinite(r)) min_front = std::min(min_front, r);
    }
    front_range_ = min_front;
    have_scan_ = true;
  }

  void tick()
  {
    geometry_msgs::msg::Twist cmd;
    if (!have_odom_ || waypoints_.empty()) {
      cmd_pub_->publish(cmd);
      return;
    }

    if (have_scan_ && front_range_ < front_stop_) {
      cmd.angular.z = 0.45;
      cmd_pub_->publish(cmd);
      return;
    }

    const auto & target = waypoints_[current_wp_];
    const double dx = target.first - x_;
    const double dy = target.second - y_;
    const double dist = std::hypot(dx, dy);
    if (dist < waypoint_tol_) {
      current_wp_ = (current_wp_ + 1) % waypoints_.size();
      if (++visited_count_ % waypoints_.size() == 0) {
        RCLCPP_INFO(get_logger(), "completed one mapping sweep; continuing for denser SLAM updates.");
      }
      cmd_pub_->publish(cmd);
      return;
    }

    const double target_yaw = std::atan2(dy, dx);
    const double yaw_error = wrap(target_yaw - yaw_);
    cmd.angular.z = std::clamp(0.95 * yaw_error, -max_angular_, max_angular_);

    if (std::abs(yaw_error) > 0.22) {
      cmd.linear.x = 0.0;
    } else {
      cmd.linear.x = linear_speed_;
      cmd.angular.z = std::clamp(0.55 * yaw_error, -0.18, 0.18);
    }
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<double, double>> waypoints_;
  size_t current_wp_ = 0;
  int visited_count_ = 0;
  bool have_odom_ = false;
  bool have_scan_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double front_range_ = std::numeric_limits<double>::infinity();
  double linear_speed_ = 0.16;
  double max_angular_ = 0.65;
  double waypoint_tol_ = 0.22;
  double front_stop_ = 0.42;
  double front_sector_rad_ = 0.45;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmallAutoMapper>());
  rclcpp::shutdown();
  return 0;
}
