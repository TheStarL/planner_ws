cmake_minimum_required(VERSION 3.8)
project(planner_gazebo_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(ackermann_teleop src/ackermann_teleop.cpp)
ament_target_dependencies(ackermann_teleop
  rclcpp ackermann_msgs)

install(TARGETS ackermann_teleop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf config worlds launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()