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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        description="World-aligned generated map by default. A saved SLAM map needs explicit frame offsets.")
    # Closed-loop execution must be world-aligned: the path, Gazebo odometry, and
    # map frame all need to describe the same coordinates. The default small_map is
    # generated from the Gazebo world, so map->odom is identity. By default the
    # physical robot and planner path both start at world/map (0, 0), then drive
    # to the upper-right free area with enough clearance from the internal boxes.
    declare_m2o_x = DeclareLaunchArgument(
        "map_to_odom_x", default_value="0.0",
        description="map->odom x offset. 0.0 for the generated world-aligned small_map.")
    declare_m2o_y = DeclareLaunchArgument(
        "map_to_odom_y", default_value="0.0",
        description="map->odom y offset. 0.0 for the generated world-aligned small_map.")
    declare_start_x = DeclareLaunchArgument(
        "start_x", default_value="0.0",
        description="Planner start x in the map frame.")
    declare_start_y = DeclareLaunchArgument(
        "start_y", default_value="0.0",
        description="Planner start y in the map frame.")
    declare_goal_x = DeclareLaunchArgument(
        "goal_x", default_value="2.35",
        description="Planner goal x in the map frame.")
    declare_goal_y = DeclareLaunchArgument(
        "goal_y", default_value="1.75",
        description="Planner goal y in the map frame.")

    spawn_x = "0.0"
    spawn_y = "0.0"
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
    # For the generated small_map, map is also the world frame, so the default is
    # identity. Saved SLAM maps generally need a non-zero offset here.
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
            "odom_frame": "odom",
        }],
        output="screen",
    )

    # Draw /planner/path as a green line inside the Gazebo 3D view. Gazebo renders
    # in world/odom coordinates, while /planner/path is in map. Since map->odom is
    # t, world = map - t, so the marker offset is -map_to_odom.
    gazebo_path_marker = Node(
        package="small_gazebo_demo",
        executable="gazebo_path_marker.py",
        name="gazebo_path_marker",
        parameters=[{
            "marker_x_offset": ParameterValue(
                PythonExpression(["-1.0 * ", map_to_odom_x]), value_type=float),
            "marker_y_offset": ParameterValue(
                PythonExpression(["-1.0 * ", map_to_odom_y]), value_type=float),
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
