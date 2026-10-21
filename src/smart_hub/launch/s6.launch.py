"""S6 routing test (owner M4, checker by M5, about 1.5 min).

    ros2 launch smart_hub s6.launch.py output_dir:=output

Starts the hub on scenario S6 (route_planner, robot_manager, traffic_monitor,
hub_coordinator, M5 stack) plus passenger_detector, so the crossing state is
valid and free (no crossing wait in the costs), and s6_check.
Evidence: <output_dir>/s6_check.json, s6_routes.csv, s6_maps.png.
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
    check = Node(package='smart_hub', executable='s6_check', name='s6_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S6', 'seed': '6', 'output_dir': out}.items()),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             output='screen',
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'scenario_id': 'S6'}]),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s6_check finished'))])),
    ])
