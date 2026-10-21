"""S9 demonstration and acceptance run (about 40 s).

    ros2 launch smart_hub s9.launch.py seed:=9 output_dir:=output

Starts the M5 stack from hub.launch.py with the e-stop published by the
operator (s9_fault_check), runs the S9 recovery sequence (T16, T17 controller
loss, reset rules), and shuts everything down when the check finishes.
Evidence: <output_dir>/s9_check.json and <output_dir>/<run_id>/.
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
    check = Node(package='smart_hub', executable='s9_fault_check', name='s9_fault_check',
                 output='screen', parameters=[{'output_dir': out}])
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='9'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S9', 'seed': LaunchConfiguration('seed'),
                              'output_dir': out, 'publish_estop': 'false',
                              # s9_fault_check plays the robot's manager itself
                              'robot': 'false', 'record_dashboard': 'true'}.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s9_fault_check finished'))])),
    ])
