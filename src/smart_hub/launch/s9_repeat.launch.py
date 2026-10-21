"""S9 fault repetition (assignment 16.3): 5 faults x 5 mission phases (~4 min).

    ros2 launch smart_hub s9_repeat.launch.py output_dir:=output

Starts the M5 stack with the e-stop published by the checker and shuts down
when s9_repeat_check finishes. Evidence: <output_dir>/s9_repeat.{json,csv}.
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
    check = Node(package='smart_hub', executable='s9_repeat_check', name='s9_repeat_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S9', 'seed': '100', 'output_dir': out,
                              'publish_estop': 'false'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s9_repeat_check finished'))])),
    ])
