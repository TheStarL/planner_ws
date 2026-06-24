"""Run the existing planner_gazebo_demo planners on the small-room map.

Usage:
  ros2 launch small_gazebo_demo planner.launch.py planner:=ego
  ros2 launch small_gazebo_demo planner.launch.py map_yaml:=/abs/path/slam_small_map.yaml planner:=bspline
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("small_gazebo_demo")
    default_map = os.path.join(pkg_share, "maps", "small_map.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "planner.rviz")
    urdf_file = os.path.join(pkg_share, "urdf", "small_diffbot.urdf.xacro")

    planner = LaunchConfiguration("planner")
    map_yaml = LaunchConfiguration("map_yaml")

    declare_planner = DeclareLaunchArgument(
        "planner", default_value="ego",
        choices=["kino_astar", "bspline", "ego"],
        description="Planner from planner_gazebo_demo to reuse.")
    declare_map = DeclareLaunchArgument(
        "map_yaml", default_value=default_map,
        description="Map from SLAM map_saver, or the generated fallback small_map.yaml.")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", urdf_file])}],
        output="screen",
    )

    path_player = Node(
        package="small_gazebo_demo",
        executable="small_path_player",
        name="small_path_player",
        parameters=[{
            "play_speed": 0.22,
            "start_x": -2.35,
            "start_y": -2.35,
            "start_yaw": 0.0,
        }],
        output="screen",
    )

    planner_node = Node(
        package="planner_gazebo_demo",
        executable=[planner, "_planner"],
        name=planner,
        parameters=[{
            "map_yaml": map_yaml,
            "start_x": -2.35,
            "start_y": -2.35,
            "start_theta": 0.0,
            "goal_x": 2.35,
            "goal_y": 2.35,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_X11_NO_MITSHM": "1",
        },
        output="screen",
    )

    return LaunchDescription([
        declare_planner,
        declare_map,
        robot_state_publisher,
        path_player,
        planner_node,
        rviz,
    ])
