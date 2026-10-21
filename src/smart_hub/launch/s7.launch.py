"""S7 robot escort test (owner M4, checker by M5, about 15 min).

    ros2 launch smart_hub s7.launch.py output_dir:=output

Starts the hub on scenario S7 (robot_manager, route_planner, traffic_monitor,
hub_coordinator, M5 stack) plus passenger_detector (a valid crossing state is
valid and free (no crossing wait in the costs), and s7_check.
Evidence: <output_dir>/s7_check.json, s7_trials.csv, s7_trace.csv, s7_events.csv, s7_paths.png.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('smart_hub')
    out = LaunchConfiguration('output_dir')
    check = Node(package='smart_hub', executable='s7_check', name='s7_check',
                 output='screen',
                 parameters=[{'output_dir': out,
                              'nominal_trials': LaunchConfiguration('nominal_trials')}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        DeclareLaunchArgument('nominal_trials', default_value='10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S7', 'seed': '7', 'output_dir': out}.items()),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             output='screen',
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'scenario_id': 'S7'}]),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s7_check finished'))])),
    ])
