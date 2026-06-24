"""
Full closed loop in Gazebo: perception map -> planning -> control execution.

  Gazebo (physics car) + controllers + tf_odometry_relay
  + static map->odom (localises the car in the world-aligned generated map)
  + a planner (kino_astar | bspline | ego) computing /planner/path on the map
  + gazebo_path_follower driving the REAL car along that path
  + RViz.

Prereq: generate the (world-aligned) map first:
  python3 src/planner_gazebo_demo/scripts/world_to_map.py

Run:
  ros2 launch planner_gazebo_demo gazebo_closed_loop.launch.py
  ros2 launch planner_gazebo_demo gazebo_closed_loop.launch.py planner:=bspline gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("planner_gazebo_demo")
    world_file = os.path.join(pkg_share, "worlds", "planner_world.sdf")
    urdf_file = os.path.join(pkg_share, "urdf", "ackermann_car.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "planner.rviz")
    default_map = os.path.join(pkg_share, "maps", "planner_map.yaml")

    # WSL2 / headless-GL software rendering (so Gazebo + RViz don't crash).
    env_libgl = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    env_mitshm = SetEnvironmentVariable("QT_X11_NO_MITSHM", "1")
    env_svga = SetEnvironmentVariable("SVGA_VGPU10", "0")

    gui = LaunchConfiguration("gui")
    planner = LaunchConfiguration("planner")
    map_yaml = LaunchConfiguration("map_yaml")
    declare_gui = DeclareLaunchArgument("gui", default_value="false",
        description="Show the Gazebo window (keep false on WSL).")
    declare_planner = DeclareLaunchArgument("planner", default_value="ego",
        choices=["kino_astar", "bspline", "ego"])
    declare_map = DeclareLaunchArgument("map_yaml", default_value=default_map,
        description="World-aligned occupancy map (generate with world_to_map.py).")

    # Spawn pose — MUST match the static map->odom transform below.
    spawn_x, spawn_y, spawn_yaw = "-7.0", "-6.0", "0.706"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_file, "verbose": "true", "gui": gui}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": True,
                     "robot_description": Command(["xacro ", urdf_file])}],
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "ackermann_car", "-topic", "robot_description",
                   "-x", spawn_x, "-y", spawn_y, "-z", "0.1", "-Y", spawn_yaw,
                   "-timeout", "120.0"],
        output="screen",
    )
    load_jsb = Node(package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen")
    load_ack = Node(package="controller_manager", executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen")
    relay = Node(package="planner_gazebo_demo", executable="tf_odometry_relay",
        name="tf_odometry_relay",
        parameters=[{"use_sim_time": True, "restamp": True, "future_offset": 0.0}],
        output="screen")

    # Localise the car in the (world-aligned) map: odom frame starts at the spawn
    # pose, so map->odom is exactly the spawn pose. Then odom->base_link (relay)
    # completes map->base_link.
    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["--x", spawn_x, "--y", spawn_y, "--z", "0.0", "--yaw", spawn_yaw,
                   "--frame-id", "map", "--child-frame-id", "odom"],
        output="screen",
    )

    planner_node = Node(
        package="planner_gazebo_demo", executable=[planner, "_planner"], name=planner,
        parameters=[{"map_yaml": map_yaml, "start_x": -7.0, "start_y": -6.0,
                     "start_theta": 0.706, "goal_x": 7.0, "goal_y": 6.0}],
        output="screen",
    )
    follower = Node(
        package="planner_gazebo_demo", executable="gazebo_path_follower",
        name="gazebo_path_follower",
        parameters=[{"use_sim_time": True, "cruise_speed": 0.5, "goal_tol": 0.6}],
        output="screen",
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config], parameters=[{"use_sim_time": True}],
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1", "QT_X11_NO_MITSHM": "1"},
        output="screen",
    )

    return LaunchDescription([
        env_libgl, env_mitshm, env_svga,
        declare_gui, declare_planner, declare_map,
        gazebo, robot_state_publisher, spawn_robot, map_to_odom,
        TimerAction(period=5.0, actions=[load_jsb]),
        TimerAction(period=7.0, actions=[load_ack]),
        TimerAction(period=8.0, actions=[relay, planner_node]),
        TimerAction(period=11.0, actions=[rviz]),
        # follower last, after controllers + relay + planner are up (TF + path ready)
        TimerAction(period=14.0, actions=[follower]),
    ])
