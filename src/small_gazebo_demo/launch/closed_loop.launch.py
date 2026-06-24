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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("small_gazebo_demo")
    world_file = os.path.join(pkg_share, "worlds", "small_room.world")
    urdf_file = os.path.join(pkg_share, "urdf", "small_diffbot.urdf.xacro")
    default_map = os.path.join(pkg_share, "maps", "slam_small_map.yaml")
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
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")

    declare_gui = DeclareLaunchArgument("gui", default_value="false")
    declare_planner = DeclareLaunchArgument(
        "planner", default_value="ego", choices=["kino_astar", "bspline", "ego"])
    declare_map = DeclareLaunchArgument(
        "map_yaml", default_value=default_map,
        description="Saved SLAM map YAML (default) or generated fallback small_map.yaml.")
    # The robot spawns at the world room corner (-2.35, -2.35). The diff-drive
    # plugin reports odometry *relative to the spawn pose*, so at t=0 odom->base_link
    # is (0, 0): the odom frame origin sits on the spawn corner. The SLAM map was
    # built starting from that same corner, so the corner is map (0, 0) too -- which
    # means map->odom is IDENTITY for the SLAM map. (A non-zero offset here shifts the
    # robot away from the path start in RViz and makes the follower stop early.)
    # The planner start (0, 0) is that corner; goal (4.7, 4.65) is the opposite corner.
    declare_m2o_x = DeclareLaunchArgument(
        "map_to_odom_x", default_value="0.0",
        description="map->odom x offset. 0.0 for the SLAM map (spawn corner = map origin).")
    declare_m2o_y = DeclareLaunchArgument(
        "map_to_odom_y", default_value="0.0",
        description="map->odom y offset. 0.0 for the SLAM map (spawn corner = map origin).")
    declare_start_x = DeclareLaunchArgument(
        "start_x", default_value="0.0",
        description="Planner start x in the map frame (robot corner + map offset).")
    declare_start_y = DeclareLaunchArgument(
        "start_y", default_value="0.0",
        description="Planner start y in the map frame (robot corner + map offset).")
    declare_goal_x = DeclareLaunchArgument(
        "goal_x", default_value="4.7",
        description="Planner goal x in the map frame (opposite corner).")
    declare_goal_y = DeclareLaunchArgument(
        "goal_y", default_value="4.65",
        description="Planner goal y in the map frame (opposite corner).")

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

    # The small robot uses Gazebo world odometry, so odom is the world frame.
    # The SLAM map frame is shifted from the world frame by map_to_odom_(x,y),
    # so map->odom must carry that offset (use 0,0 for the world-aligned
    # small_map). Without it the planner start lands off the SLAM map and the
    # robot starts in the wrong place.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="small_map_to_odom",
        arguments=[
            "--x", map_to_odom_x, "--y", map_to_odom_y, "--z", "0.0",
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
            "start_x": ParameterValue(start_x, value_type=float),
            "start_y": ParameterValue(start_y, value_type=float),
            "start_theta": 0.0,
            "goal_x": ParameterValue(goal_x, value_type=float),
            "goal_y": ParameterValue(goal_y, value_type=float),
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

    # Draws /planner/path as a green line inside the Gazebo 3D view. The path is in
    # the map frame; Gazebo renders in the world frame. The map origin (0,0) is the
    # spawn corner in the world, so world = map + (spawn_x, spawn_y). Only with gui:=true.
    gazebo_path_marker = Node(
        package="small_gazebo_demo",
        executable="gazebo_path_marker.py",
        name="gazebo_path_marker",
        parameters=[{
            "marker_x_offset": float(spawn_x),
            "marker_y_offset": float(spawn_y),
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
        declare_m2o_x, declare_m2o_y,
        declare_start_x, declare_start_y, declare_goal_x, declare_goal_y,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        map_to_odom,
        TimerAction(period=5.0, actions=[planner_node]),
        TimerAction(period=7.0, actions=[rviz]),
        TimerAction(period=8.0, actions=[follower, gazebo_path_marker]),
    ])
