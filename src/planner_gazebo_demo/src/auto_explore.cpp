// auto_explore.cpp
//
// Hands-free exploration for SLAM mapping, anchored at the start point.
// Includes an automated "Reverse Recovery" mechanism when stuck.

#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AutoExplore : public rclcpp::Node
{
public:
  AutoExplore() : Node("auto_explore")
  {
    cruise_        = this->declare_parameter("cruise_speed", 0.4);   // m/s
    max_range_     = this->declare_parameter("max_range", 9.0);      // m: turn back when this far from start
    min_range_     = this->declare_parameter("min_range", 2.0);      // m: "back at start" radius
    
    // 扇形展开 90 度，中心朝向 45 度（右上方第一象限）
    sweep_deg_     = this->declare_parameter("sweep_deg", 90.0);
    center_deg_    = this->declare_parameter("center_deg", 45.0);
    
    num_spokes_    = this->declare_parameter("num_spokes", 8);
    wheelbase_     = this->declare_parameter("wheelbase", 0.9);
    steer_max_     = this->declare_parameter("steer_max", 0.6109);   // 35 deg
    min_lookahead_ = this->declare_parameter("min_lookahead", 1.5);
    
    spawn_x_       = this->declare_parameter("spawn_x", -7.0);
    spawn_y_       = this->declare_parameter("spawn_y", -6.0);
    spawn_yaw_     = this->declare_parameter("spawn_yaw", 0.706);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_steering_controller/reference_unstamped", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ackermann_steering_controller/odometry", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {odom_ = m; have_odom_ = true;});
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&AutoExplore::tick, this));

    theta_out_ = spokeAngle(0);
    RCLCPP_INFO(this->get_logger(),
      "auto_explore: spoke explorer with Reverse Recovery. max_range=%.1f m, cruise=%.2f m/s.",
      max_range_, cruise_);
  }

private:
  double spokeAngle(int i) const
  {
    const double center = center_deg_ * M_PI / 180.0;
    if (num_spokes_ <= 1) {return center;}
    if (sweep_deg_ >= 359.9) {
      return wrap(center + i * (2.0 * M_PI / num_spokes_));
    }
    const double h = (sweep_deg_ * M_PI / 180.0) / 2.0;
    const double frac = double(i) / double(num_spokes_ - 1);
    return wrap(center - h + frac * (2.0 * h));
  }
  
  static double wrap(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }
  
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void tick()
  {
    geometry_msgs::msg::Twist cmd;
    if (!have_odom_) {pub_->publish(cmd); return;}

    const double ox = odom_->pose.pose.position.x;
    const double oy = odom_->pose.pose.position.y;
    const double oyaw = yawFromQuat(odom_->pose.pose.orientation);
    const double d = std::hypot(ox, oy);

    // --- 1. 卡死检测 (Anti-stuck) ---
    bool stuck = false;
    if (++stuck_ticks_ >= 60) { // 每 3 秒检查一次
      stuck = (std::hypot(ox - last_x_, oy - last_y_) < 0.25);
      last_x_ = ox; last_y_ = oy; stuck_ticks_ = 0;
    }

    // ⭐ 新增：触发倒车脱困模式
    if (stuck) {
      reverse_ticks_ = 30; // 触发倒车，持续 30 个 tick (即 1.5 秒)
      RCLCPP_WARN(this->get_logger(), "检测到卡死！启动自动倒车...");
    }

    // ⭐ 新增：执行倒车动作
    // 如果处于倒车状态，优先执行倒车，直接跳过后续的纯追踪规划
    if (reverse_ticks_ > 0) {
      reverse_ticks_--;
      cmd.linear.x = -0.25; // 挂倒挡：负速度后退
      cmd.angular.z = 0.0;  // 倒车时方向盘回正（直直往后退）
      pub_->publish(cmd);
      return; 
    }

    // --- 2. 状态切换 (Phase transitions) ---
    if (outbound_) {
      if (d >= max_range_ || stuck) {
        outbound_ = false; // 离起点太远或撞墙，开始返回
      }
    } else {
      if (d <= min_range_ || stuck) {
        spoke_ = (spoke_ + 1) % num_spokes_; // 回到起点附近，换下一个方向
        theta_out_ = spokeAngle(spoke_);
        outbound_ = true;
      }
    }

    // --- 3. 目标点设定 ---
    double tx, ty;
    if (outbound_) {
      tx = (max_range_ + 2.0) * std::cos(theta_out_);
      ty = (max_range_ + 2.0) * std::sin(theta_out_);
    } else {
      tx = 0.0; ty = 0.0;
    }

    // --- 4. 纯追踪控制与防滑逻辑 ---
    const double alpha = wrap(std::atan2(ty - oy, tx - ox) - oyaw);
    double steer;
    
    // 目标在身后时，方向盘打死
    if (std::abs(alpha) > 1.2) {
      steer = (alpha > 0.0) ? steer_max_ : -steer_max_;
    } else {
      const double Ld = std::max(std::hypot(tx - ox, ty - oy), min_lookahead_);
      steer = std::clamp(std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld),
                         -steer_max_, steer_max_);
    }

    // 防滑降速逻辑
    double steer_ratio = std::abs(steer) / steer_max_;
    double speed_penalty = 1.0 - 0.85 * steer_ratio; 
    
    cmd.linear.x = cruise_ * speed_penalty;
    if (cmd.linear.x < 0.05) {
        cmd.linear.x = 0.05;
    }

    cmd.angular.z = cmd.linear.x * std::tan(steer) / wheelbase_;
    pub_->publish(cmd);

    // --- 5. 日志打印 ---
    if (++log_ticks_ >= 40) {
      log_ticks_ = 0;
      RCLCPP_INFO(this->get_logger(),
        "%s spoke#%d(%.0f deg) | dist=%.2f m | steer=%.2f",
        outbound_ ? "OUT " : "BACK", spoke_, spokeAngle(spoke_) * 180.0 / M_PI, d, steer);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  bool have_odom_ = false;

  double cruise_, max_range_, min_range_, sweep_deg_, center_deg_;
  int num_spokes_;
  double wheelbase_, steer_max_, min_lookahead_;
  double spawn_x_, spawn_y_, spawn_yaw_;

  bool outbound_ = true;
  int spoke_ = 0;
  double theta_out_ = 0.0;
  int stuck_ticks_ = 0, log_ticks_ = 0;
  double last_x_ = 0.0, last_y_ = 0.0;
  
  // ⭐ 新增：倒车状态计数器
  int reverse_ticks_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoExplore>());
  rclcpp::shutdown();
  return 0;
}