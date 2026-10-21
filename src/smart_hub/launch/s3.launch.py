"""S3 demonstration and acceptance run (owner M2-A, about 12 min).

    ros2 launch smart_hub s3.launch.py output_dir:=output

Starts the hub from hub.launch.py on scenario S3 (bus_manager, hub_coordinator
and the M5 stack), passenger_detector on the simulator camera (the crossing
is only granted with a valid pedestrian observation) and s3_check, which
runs the 15 trials (T05, T06). M1-B's demand_predictor is not started, so
the coordinator dispatches only on the observed queue (the last trial).
Shuts everything down when the check finishes.
Evidence: <output_dir>/s3_check.json, s3_trials.csv, s3_trace.csv,
s3_events.csv, s3_paths.png.
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
    check = Node(package='smart_hub', executable='s3_check', name='s3_check',
                 output='screen',
                 parameters=[{'output_dir': out,
                              'nominal_trials': LaunchConfiguration('nominal_trials')}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        DeclareLaunchArgument('nominal_trials', default_value='10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S3', 'seed': '1', 'output_dir': out,
                              'bus': 'true', 'coordination': 'true'}.items()),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             output='screen',
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'scenario_id': 'S3'}]),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s3_check finished'))])),
    ])
