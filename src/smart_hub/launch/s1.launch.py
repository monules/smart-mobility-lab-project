"""S1 demonstration and acceptance run (about 45 s), owner M1-A.

    ros2 launch smart_hub s1.launch.py detector:=blob output_dir:=output

Starts the M5 stack (simulator with the synthetic camera) on scenario S1,
passenger_detector and s1_check; shuts down when the check finishes.
Evidence: <output_dir>/s1_check.json.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('smart_hub')
    out = LaunchConfiguration('output_dir')
    detector = LaunchConfiguration('detector')
    check = Node(package='smart_hub', executable='s1_check', name='s1_check',
                 output='screen',
                 parameters=[{'output_dir': out,
                              # exact counts are only claimed for the blob baseline
                              'exact_counts': PythonExpression(["'", detector, "' == 'blob'"])}])
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('detector', default_value='blob'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S1', 'seed': LaunchConfiguration('seed'),
                              'output_dir': out}.items()),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             output='screen',
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'detector': detector, 'scenario_id': 'S1'}]),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s1_check finished'))])),
    ])
