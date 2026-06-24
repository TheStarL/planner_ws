#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SmallScanFilter : public rclcpp::Node
{
public:
  SmallScanFilter()
  : Node("small_scan_filter")
  {
    input_topic_ = declare_parameter("input_topic", std::string("/scan_raw"));
    output_topic_ = declare_parameter("output_topic", std::string("/scan"));
    max_abs_angular_ = declare_parameter("max_abs_angular", 0.10);
    max_abs_lateral_ = declare_parameter("max_abs_lateral", 0.03);
    min_forward_speed_ = declare_parameter("min_forward_speed", 0.015);
    max_forward_speed_ = declare_parameter("max_forward_speed", 0.16);
    max_range_ = declare_parameter("max_range", 4.5);
    max_odom_age_ = declare_parameter("max_odom_age", 0.35);
    publish_static_scans_ = declare_parameter("publish_static_scans", true);
    static_scan_period_ = declare_parameter("static_scan_period", 0.8);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&SmallScanFilter::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SmallScanFilter::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "small_scan_filter: %s -> %s, max_abs_angular=%.3f rad/s",
      input_topic_.c_str(), output_topic_.c_str(), max_abs_angular_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    linear_x_ = msg->twist.twist.linear.x;
    linear_y_ = msg->twist.twist.linear.y;
    angular_z_ = msg->twist.twist.angular.z;
    last_odom_stamp_ = now();
    have_odom_ = true;
  }

  bool shouldPublish() const
  {
    if (!have_odom_) {
      return false;
    }

    const double odom_age = (now() - last_odom_stamp_).seconds();
    if (odom_age > max_odom_age_) {
      return false;
    }

    const bool stable_heading = std::abs(angular_z_) <= max_abs_angular_;
    const bool low_lateral = std::abs(linear_y_) <= max_abs_lateral_;
    const bool forward = linear_x_ >= min_forward_speed_ && linear_x_ <= max_forward_speed_;
    const bool static_scan_due = !have_static_scan_stamp_ ||
      (now() - last_static_scan_stamp_).seconds() >= static_scan_period_;
    const bool static_ok = publish_static_scans_ && std::abs(linear_x_) < min_forward_speed_ &&
      static_scan_due;

    return stable_heading && low_lateral && (forward || static_ok);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!shouldPublish()) {
      if (++drop_log_count_ % 80 == 1) {
        RCLCPP_INFO(
          get_logger(),
          "dropping scan during turn/recovery: vx=%.3f wz=%.3f",
          linear_x_, angular_z_);
      }
      return;
    }

    auto out = *msg;
    out.range_max = std::min(out.range_max, static_cast<float>(max_range_));
    for (auto & r : out.ranges) {
      if (!std::isfinite(r)) {
        continue;
      }
      if (r > out.range_max) {
        r = std::numeric_limits<float>::infinity();
      }
    }

    if (std::abs(linear_x_) < min_forward_speed_) {
      last_static_scan_stamp_ = now();
      have_static_scan_stamp_ = true;
    }
    scan_pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string input_topic_;
  std::string output_topic_;
  bool have_odom_ = false;
  bool have_static_scan_stamp_ = false;
  bool publish_static_scans_ = true;
  int drop_log_count_ = 0;
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_z_ = 0.0;
  double max_abs_angular_ = 0.10;
  double max_abs_lateral_ = 0.03;
  double min_forward_speed_ = 0.015;
  double max_forward_speed_ = 0.16;
  double max_range_ = 4.5;
  double max_odom_age_ = 0.35;
  double static_scan_period_ = 0.8;
  rclcpp::Time last_odom_stamp_;
  rclcpp::Time last_static_scan_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmallScanFilter>());
  rclcpp::shutdown();
  return 0;
}
