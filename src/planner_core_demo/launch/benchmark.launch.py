"""Run the head-less planning-framework benchmark and dump a CSV.

Usage:
    ros2 launch planner_core_demo benchmark.launch.py
    ros2 launch planner_core_demo benchmark.launch.py num_trials:=50 output_csv:=/tmp/result.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_trials_arg = DeclareLaunchArgument(
        'num_trials', default_value='30', description='Number of randomised trials')
    output_csv_arg = DeclareLaunchArgument(
        'output_csv', default_value='benchmark_results.csv',
        description='Path of the CSV result file')

    benchmark_node = Node(
        package='planner_core_demo',
        executable='planner_benchmark',
        name='planner_benchmark',
        output='screen',
        parameters=[{
            'num_trials': LaunchConfiguration('num_trials'),
            'output_csv': LaunchConfiguration('output_csv'),
        }],
    )

    return LaunchDescription([num_trials_arg, output_csv_arg, benchmark_node])
