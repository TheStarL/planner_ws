import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'planner_gazebo_demo'

    # ── Paths ─────────────────────────────────────────────────────
    pkg_share = get_package_share_directory(pkg)
    urdf_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'planner_world.sdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'gazebo.rviz')
    controller_config = os.path.join(pkg_share, 'config', 'ackermann_control.yaml')

    # ── Use sim time ──────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Robot description (xacro → urdf) ──────────────────────────
    robot_description = {
        'robot_description': Command(['xacro ', urdf_file, ' use_sim:=true'])
    }

    # ── 1. Gazebo ─────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen')

    # ── 2. Spawn the Ackermann car ────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'ackermann_car', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.3'],
        output='screen')

    # ── 3. robot_state_publisher ──────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen')

    # ── 4. Controller manager + spawners ──────────────────────────
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[robot_description, controller_config,
                    {'use_sim_time': use_sim_time}],
        output='screen')

    # Delay spawners so controller_manager is ready
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '--param-file', controller_config],
            output='screen'
        )]
    )

    ackermann_controller_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['ackermann_steering_controller', '--param-file', controller_config],
            output='screen'
        )]
    )

    # ── 5. RViz2 ──────────────────────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ── 6. Ackermann teleop node ──────────────────────────────────
    teleop = Node(
        package=pkg, executable='ackermann_teleop',
        name='ackermann_teleop', output='screen',
        prefix='xterm -e'  # runs in its own terminal for keyboard input
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) time'),

        gazebo,
        robot_state_pub,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
        rviz,
        teleop,
    ])