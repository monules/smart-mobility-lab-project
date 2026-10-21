"""S8 integrated rush hour, one episode (team; scenario and checker by M5, ~4.5 min).

    ros2 launch smart_hub s8.launch.py seed:=1 dispatch:=predictive output_dir:=output/s8

Starts every node of the hub on scenario S8: hub.launch.py with perception
(camera -> passenger_detector -> demand_predictor), the coordinator in the
given dispatch mode (predictive or reactive), the bus departing to exit E
after docking, and s8_check. Shuts everything down when the episode ends.
docker/hub.sh s8 runs ten seeds in both modes and summarises them (T15).
Evidence: <output_dir>/s8_episode.json, s8_timeline.csv (+ dashboard frames
when record_dashboard:=true).
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
    # 'dispatch', not 'mode': hub.launch.py already has 'mode' (sim/hardware) and
    # launch arguments share one scope, so 'mode:=predictive' would reach the
    # actuator bridge (which then refuses to start - found in the first S8 run)
    out, seed, mode = (LaunchConfiguration(n) for n in ('output_dir', 'seed', 'dispatch'))
    check = Node(package='smart_hub', executable='s8_check', name='s8_check',
                 output='screen',
                 parameters=[{'output_dir': out, 'seed': seed, 'mode': mode}])
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('dispatch', default_value='predictive'),
        DeclareLaunchArgument('output_dir', default_value='output/s8'),
        DeclareLaunchArgument('record_dashboard', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S8', 'seed': seed, 'output_dir': out,
                              'perception': 'true', 'dispatch_mode': mode,
                              'bus_depart_to': 'E',
                              'record_dashboard': LaunchConfiguration('record_dashboard')
                              }.items()),
        check,
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='s8_check finished'))])),
    ])
