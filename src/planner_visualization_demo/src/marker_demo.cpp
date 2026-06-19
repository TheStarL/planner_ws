#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class MarkerDemoNode : public rclcpp::Node
{
public:
  MarkerDemoNode() : Node("marker_demo_node")
  {
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planner_demo/markers", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&MarkerDemoNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "planner_visualization_demo started.");
  }

private:
  void timerCallback()
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    path.ns = "demo_path";
    path.id = 0;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;

    path.scale.x = 0.06;

    path.color.r = 0.1;
    path.color.g = 0.8;
    path.color.b = 0.2;
    path.color.a = 1.0;

    for (int i = 0; i < 100; ++i) {
      double x = -5.0 + 0.1 * i;
      double y = std::sin(x);
      double z = 1.0 + 0.2 * std::cos(2.0 * x);

      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      path.points.push_back(p);
    }

    visualization_msgs::msg::Marker start;
    start.header.frame_id = "map";
    start.header.stamp = this->now();
    start.ns = "demo_points";
    start.id = 1;
    start.type = visualization_msgs::msg::Marker::SPHERE;
    start.action = visualization_msgs::msg::Marker::ADD;
    start.pose.position.x = -5.0;
    start.pose.position.y = std::sin(-5.0);
    start.pose.position.z = 1.0 + 0.2 * std::cos(-10.0);
    start.scale.x = 0.25;
    start.scale.y = 0.25;
    start.scale.z = 0.25;
    start.color.r = 0.0;
    start.color.g = 1.0;
    start.color.b = 0.0;
    start.color.a = 1.0;

    visualization_msgs::msg::Marker goal = start;
    goal.id = 2;
    goal.pose.position.x = 4.9;
    goal.pose.position.y = std::sin(4.9);
    goal.pose.position.z = 1.0 + 0.2 * std::cos(9.8);
    goal.color.r = 1.0;
    goal.color.g = 0.0;
    goal.color.b = 0.0;
    goal.color.a = 1.0;

    array.markers.push_back(path);
    array.markers.push_back(start);
    array.markers.push_back(goal);

    pub_->publish(array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerDemoNode>());
  rclcpp::shutdown();
  return 0;
}
