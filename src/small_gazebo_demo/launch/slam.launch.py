"""Stable small-world Gazebo -> SLAM -> RViz pipeline.

Usage:
  ros2 launch small_gazebo_demo slam.launch.py
  ros2 launch small_gazebo_demo slam.launch.py gui:=true auto_drive:=false

After the map is complete:
  ros2 run nav2_map_server map_saver_cli -f ~/planner_ws/src/small_gazebo_demo/maps/slam_small_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("small_gazebo_demo")
    world_file = os.path.join(pkg_share, "worlds", "small_room.world")
    urdf_file = os.path.join(pkg_share, "urdf", "small_diffbot.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "slam.rviz")

    env_libgl = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    env_mitshm = SetEnvironmentVariable("QT_X11_NO_MITSHM", "1")
    env_svga = SetEnvironmentVariable("SVGA_VGPU10", "0")
    env_home = SetEnvironmentVariable("HOME", "/tmp")
    env_gazebo_ip = SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1")
    env_gazebo_master = SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345")

    gui = LaunchConfiguration("gui")
    auto_drive = LaunchConfiguration("auto_drive")
    rviz = LaunchConfiguration("rviz")

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="false",
        description="Show Gazebo GUI. Keep false for WSL/headless stability.")
    declare_auto = DeclareLaunchArgument(
        "auto_drive", default_value="true",
        description="Run the slow deterministic mapping sweep.")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Start RViz with /map, /scan, TF and RobotModel.")

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
            "-x", "-2.35",
            "-y", "-2.35",
            "-z", "0.05",
            "-Y", "0.0",
            "-timeout", "90.0",
        ],
        output="screen",
    )

    slam_params = {
        "use_sim_time": True,
        "mode": "mapping",
        "base_frame": "base_link",
        "odom_frame": "odom",
        "map_frame": "map",
        "scan_topic": "/scan",
        "resolution": 0.05,
        "max_laser_range": 6.0,
        "min_laser_range": 0.12,
        "minimum_time_interval": 0.15,
        "transform_publish_period": 0.05,
        "map_update_interval": 0.5,
        "transform_timeout": 0.8,
        "minimum_travel_distance": 0.05,
        "minimum_travel_heading": 0.05,
        "scan_queue_size": 20,
        "throttle_scans": 1,
        "loop_search_maximum_distance": 4.0,
        "loop_match_minimum_chain_size": 8,
        "loop_match_maximum_variance": 0.45,
        "loop_match_minimum_response_coarse": 0.60,
        "loop_match_minimum_response_fine": 0.70,
    }

    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
    )

    mapper = Node(
        package="small_gazebo_demo",
        executable="small_auto_mapper",
        name="small_auto_mapper",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(auto_drive),
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_X11_NO_MITSHM": "1",
        },
        output="screen",
    )

    return LaunchDescription([
        env_libgl, env_mitshm, env_svga, env_home, env_gazebo_ip, env_gazebo_master,
        declare_gui, declare_auto, declare_rviz,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        TimerAction(period=4.0, actions=[slam]),
        TimerAction(period=6.0, actions=[mapper]),
        TimerAction(period=7.0, actions=[rviz_node]),
    ])
