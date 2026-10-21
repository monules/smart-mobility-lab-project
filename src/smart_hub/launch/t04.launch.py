"""T04 dispatch test (owner M2-B, checker by M5, about 4 min).

    ros2 launch smart_hub t04.launch.py output_dir:=output

Starts the hub from hub.launch.py on scenario S3 (hub_coordinator,
bus_manager and the M5 stack). t04_check publishes /hub/demand and
/hub/queue itself, so neither passenger_detector nor demand_predictor runs.
Shuts everything down when the check finishes.
Evidence: <output_dir>/t04_check.json, t04_timeline.csv.
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
    check = Node(package='smart_hub', executable='t04_check', name='t04_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S3', 'seed': '1', 'output_dir': out,
                              'bus': 'true', 'perception': 'false'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='t04_check finished'))])),
    ])
