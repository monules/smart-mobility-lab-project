"""S5 demonstration and acceptance run (owner M3, checker M5).

    ros2 launch smart_hub s5.launch.py seed:=5 output_dir:=output

Starts the hub from hub.launch.py on scenario S5 (traffic_monitor included)
and s5_check, which listens to the edge labels while the scripted traffic,
freeze and obstacle events play out (T09, T10). Shuts everything down when
the check finishes (about 90 s).
Evidence: <output_dir>/s5_check.json, s5_timeseries.csv, s5_labels.png.
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
    check = Node(package='smart_hub', executable='s5_check', name='s5_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='5'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S5', 'seed': LaunchConfiguration('seed'),
                              'output_dir': out, 'record_dashboard': 'true'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s5_check finished'))])),
    ])
