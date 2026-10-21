"""S4 demonstration and acceptance run (owner M3, checker M5).

    ros2 launch smart_hub s4.launch.py seed:=4 output_dir:=output

Starts the hub from hub.launch.py on scenario S4 (parking_manager included)
and s4_check, which plays the cars C1, C2, ... and checks reservations,
idempotency, expiry and the 20-pair concurrency test (T07, T08). Shuts
everything down when the check finishes (about 45 s).
Evidence: <output_dir>/s4_check.json.
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
    check = Node(package='smart_hub', executable='s4_check', name='s4_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='4'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S4', 'seed': LaunchConfiguration('seed'),
                              'output_dir': out, 'record_dashboard': 'true'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s4_check finished'))])),
    ])
