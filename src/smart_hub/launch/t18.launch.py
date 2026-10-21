"""T18 planner-loss test (owners M4 + M5, about 1 min).

    ros2 launch smart_hub t18.launch.py output_dir:=output

Starts the hub on scenario S9 (robot_1 at R, bus_1 at A) WITHOUT
route_planner; t18_check leaves /route/compute absent, then serves it late,
then slow but in time. Shuts everything down when the check finishes.
Evidence: <output_dir>/t18_check.json.
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
    check = Node(package='smart_hub', executable='t18_check', name='t18_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S9', 'seed': '18', 'output_dir': out,
                              'planner': 'false'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='t18_check finished'))])),
    ])
