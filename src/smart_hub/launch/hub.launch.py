"""M5 infrastructure launch: simulator, safety, actuator bridge, logger, dashboard.

    ros2 launch smart_hub hub.launch.py scenario:=S9 seed:=9 mode:=sim

Arguments (assignment 20.1: scenario, seed and simulation/hardware mode):
  scenario       scenario file name in share/smart_hub/scenarios (S9)
  seed           random seed for the simulator (9)
  mode           only 'sim' is implemented; the bridge refuses anything else
  output_dir     where metrics_logger writes <run_id>/events.csv, summary.json
  publish_estop  true: scenario_runner publishes /hub/estop (10 Hz heartbeat);
                 false: an operator node publishes it instead

Teammates' nodes (detector, predictor, managers, coordinator, planner) are
added here once they are merged; this branch contains only M5's nodes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('smart_hub'), 'config', 'm5_safety.yaml')
    scenario = LaunchConfiguration('scenario')
    seed = LaunchConfiguration('seed')
    mode = LaunchConfiguration('mode')
    out = LaunchConfiguration('output_dir')
    estop = LaunchConfiguration('publish_estop')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S9'),
        DeclareLaunchArgument('seed', default_value='9'),
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        DeclareLaunchArgument('publish_estop', default_value='true'),
        Node(package='smart_hub', executable='scenario_runner', name='scenario_runner',
             output='screen',
             parameters=[{'scenario': scenario, 'seed': seed, 'publish_estop': estop}]),
        Node(package='smart_hub', executable='safety_supervisor', name='safety_supervisor',
             output='screen', parameters=[cfg, {'scenario_id': scenario}]),
        Node(package='smart_hub', executable='actuator_bridge', name='actuator_bridge',
             output='screen', parameters=[cfg, {'mode': mode}]),
        Node(package='smart_hub', executable='metrics_logger', name='metrics_logger',
             output='screen',
             parameters=[{'output_dir': out, 'scenario_id': scenario, 'seed': seed}]),
        Node(package='smart_hub', executable='dashboard_marker_publisher',
             name='dashboard_marker_publisher', output='screen'),
    ])
