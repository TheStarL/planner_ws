"""Small Gazebo full loop: map -> planner -> diff-drive execution -> RViz.

This launch uses the saved/generated map and drives the simple Gazebo robot along
the planner path. Generate a live SLAM map first with slam.launch.py, or use the
checked-in generated fallback map.

Usage:
  ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego
  ros2 launch small_gazebo_demo closed_loop.launch.py planner:=bspline gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("small_gazebo_demo")
    world_file = os.path.join(pkg_share, "worlds", "small_room.world")
    urdf_file = os.path.join(pkg_share, "urdf", "small_diffbot.urdf.xacro")
    default_map = os.path.join(pkg_share, "maps", "small_map.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "planner.rviz")

    env_libgl = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    env_mitshm = SetEnvironmentVariable("QT_X11_NO_MITSHM", "1")
    env_svga = SetEnvironmentVariable("SVGA_VGPU10", "0")
    env_home = SetEnvironmentVariable("HOME", "/tmp")
    env_gazebo_ip = SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1")
    env_gazebo_master = SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345")

    gui = LaunchConfiguration("gui")
    planner = LaunchConfiguration("planner")
    map_yaml = LaunchConfiguration("map_yaml")

    declare_gui = DeclareLaunchArgument("gui", default_value="false")
    declare_planner = DeclareLaunchArgument(
        "planner", default_value="ego", choices=["kino_astar", "bspline", "ego"])
    declare_map = DeclareLaunchArgument(
        "map_yaml", default_value=default_map,
        description="Saved SLAM map YAML or generated fallback map.")

    spawn_x = "-2.35"
    spawn_y = "-2.35"
    spawn_yaw = "0.0"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_file, "verbose": "false", "gui": gui}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command(["xacro ", urdf_file]),
        }],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "small_diffbot",
            "-topic", "robot_description",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.05",
            "-Y", spawn_yaw,
            "-timeout", "90.0",
        ],
        output="screen",
    )

    # The small robot uses Gazebo world odometry, so odom already matches the
    # generated small-room map frame. Keep map->odom identity; applying the spawn
    # offset here would double-shift the robot in RViz and in the path follower.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="small_map_to_odom",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
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

    follower = Node(
        package="small_gazebo_demo",
        executable="small_path_follower",
        name="small_path_follower",
        parameters=[{
            "use_sim_time": True,
            "cruise_speed": 0.18,
            "lookahead": 0.65,
            "goal_tol": 0.24,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_X11_NO_MITSHM": "1",
        },
        output="screen",
    )

    return LaunchDescription([
        env_libgl, env_mitshm, env_svga, env_home, env_gazebo_ip, env_gazebo_master,
        declare_gui, declare_planner, declare_map,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        map_to_odom,
        TimerAction(period=5.0, actions=[planner_node]),
        TimerAction(period=7.0, actions=[rviz]),
        TimerAction(period=8.0, actions=[follower]),
    ])
