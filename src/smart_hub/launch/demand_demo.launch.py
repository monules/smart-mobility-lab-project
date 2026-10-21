"""Standalone S2 demo: synthetic /hub/queue replay + demand_predictor.

    ros2 launch smart_hub demand_demo.launch.py seed:=1025 run_id:=demo-01

Use this until passenger_detector (S1) exists. In the integrated hub launch,
include only demand_predictor and let passenger_detector publish /hub/queue.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('smart_hub'), 'config',
                       'demand_predictor.yaml')
    seed = LaunchConfiguration('seed')
    run_id = LaunchConfiguration('run_id')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='1025'),
        DeclareLaunchArgument('run_id', default_value='s2-demo'),
        Node(package='smart_hub', executable='fake_queue_publisher',
             name='fake_queue_publisher', output='screen',
             parameters=[{'seed': seed}]),
        Node(package='smart_hub', executable='demand_predictor',
             name='demand_predictor', output='screen',
             parameters=[cfg, {'run_id': run_id, 'scenario_id': 'S2'}]),
    ])
