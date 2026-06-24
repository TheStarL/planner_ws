cmake_minimum_required(VERSION 3.8)
project(small_gazebo_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(small_auto_mapper src/small_auto_mapper.cpp)
ament_target_dependencies(small_auto_mapper
  rclcpp geometry_msgs nav_msgs sensor_msgs)

add_executable(small_path_follower src/small_path_follower.cpp)
ament_target_dependencies(small_path_follower
  rclcpp geometry_msgs nav_msgs tf2_ros)

add_executable(small_path_player src/small_path_player.cpp)
ament_target_dependencies(small_path_player
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros)

install(TARGETS
  small_auto_mapper
  small_path_follower
  small_path_player
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  rviz
  scripts
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/small_world_to_map.py
  scripts/gazebo_path_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
