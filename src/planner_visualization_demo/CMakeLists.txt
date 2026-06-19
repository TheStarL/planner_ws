cmake_minimum_required(VERSION 3.8)
project(planner_visualization_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(marker_demo src/marker_demo.cpp)

ament_target_dependencies(marker_demo
  rclcpp
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  marker_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
