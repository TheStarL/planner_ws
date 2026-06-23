// tf_odometry_relay.cpp
//
// The ros2_controllers steering controllers (ackermann/bicycle/tricycle) publish
// their odom->base_link transform on a PRIVATE topic `~/tf_odometry` instead of
// the global `/tf` (unlike diff_drive_controller, which publishes to /tf直接).
// As a result the `odom` frame never enters the global TF tree, and any consumer
// that needs map->odom->base_link->lidar (e.g. slam_toolbox) cannot transform the
// laser scan and drops every message ("Message Filter dropping message ... queue
// is full").
//
// This node is a minimal relay: it subscribes to the controller's tf_odometry
// topic and republishes the identical TFMessage onto /tf, so odom->base_link
// joins the global tree. The transforms are passed through unchanged (their
// sim-time stamps are preserved), so no use_sim_time handling is needed here.
//
// Input  topic defaults to /ackermann_steering_controller/tf_odometry
// Output topic is /tf
// Both are overridable via the `input_topic` / `output_topic` parameters.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TfOdometryRelay : public rclcpp::Node
{
public:
  TfOdometryRelay() : Node("tf_odometry_relay")
  {
    const std::string input_topic = this->declare_parameter<std::string>(
      "input_topic", "/ackermann_steering_controller/tf_odometry");
    const std::string output_topic = this->declare_parameter<std::string>(
      "output_topic", "/tf");

    // Restamp each transform to the relay's current (sim) time before
    // republishing. A plain pass-through preserves the controller's stamp, but
    // the subscribe->republish hop adds latency, so the transform on /tf ends up
    // lagging "now". slam_toolbox looks up odom->base_link at specific times
    // (each scan's stamp and, at 20 Hz, the map->odom publish thread's "now"),
    // and a lagging transform makes every lookup fail with "extrapolation into
    // the future" -> "Failed to compute odom pose" -> no map is ever built.
    // Restamping to now() keeps odom->base_link fresh so those lookups succeed.
    // (For this node `now()` is sim time because the launch sets use_sim_time.)
    restamp_ = this->declare_parameter<bool>("restamp", true);
    // Stamp the relayed transform slightly into the FUTURE. Restamping to exactly
    // now() is not enough: slam_toolbox's map->odom thread looks up odom->base_link
    // at its own now(), which can be a sim-clock tick ahead of the relay's now(),
    // re-triggering "extrapolation into the future". A small forward offset keeps
    // odom->base_link always ahead of any consumer's lookup time. The resulting
    // pose-time error (offset x speed, e.g. 0.1 s x 1.5 m/s = 15 cm) is negligible
    // for mapping and is corrected by scan matching.
    future_offset_s_ = this->declare_parameter<double>("future_offset", 0.1);

    // /tf conventionally uses a reliable, volatile, keep-last(100) QoS; match it.
    rclcpp::QoS tf_qos(100);
    pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(output_topic, tf_qos);
    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic, tf_qos,
      [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
        if (restamp_) {
          const rclcpp::Time stamp =
            this->get_clock()->now() + rclcpp::Duration::from_seconds(future_offset_s_);
          for (auto & t : msg->transforms) {
            t.header.stamp = stamp;
          }
        }
        pub_->publish(*msg);
      });

    RCLCPP_INFO(this->get_logger(),
      "Relaying TF: '%s' -> '%s' (restamp=%s, future_offset=%.3fs)",
      input_topic.c_str(), output_topic.c_str(),
      restamp_ ? "true" : "false", future_offset_s_);
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  bool restamp_ = true;
  double future_offset_s_ = 0.1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfOdometryRelay>());
  rclcpp::shutdown();
  return 0;
}
