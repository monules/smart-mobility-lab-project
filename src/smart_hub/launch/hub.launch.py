"""M5 infrastructure launch: simulator, safety, actuator bridge, logger, dashboard.

    ros2 launch smart_hub hub.launch.py scenario:=S9 seed:=9 mode:=sim

Arguments (assignment 20.1: scenario, seed and simulation/hardware mode):
  scenario       scenario file name in share/smart_hub/scenarios (S9)
  seed           random seed for the simulator (9)
  mode           only 'sim' is implemented; the bridge refuses anything else
  output_dir     where metrics_logger writes <run_id>/events.csv, summary.json
  record_dashboard  true: save /hub/markers as PNGs + MP4 in <output_dir>/dashboard
  coordination   true (default): also start hub_coordinator (M2-B),
                 parking_manager and traffic_monitor (M3)
  bus            true (default): also start bus_manager (M2-A, DockBus server)
  perception     true: also start passenger_detector (M1-A) and demand_predictor
                 (M1-B) on the simulator camera; use with scenario:=S1
  publish_estop  true: scenario_runner publishes /hub/estop (10 Hz heartbeat);
                 false: an operator node publishes it instead

Remaining teammates' nodes (robot manager, route planner) are added here once
they are inside src/smart_hub.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('smart_hub')
    cfg = os.path.join(share, 'config', 'm5_safety.yaml')
    scenario = LaunchConfiguration('scenario')
    seed = LaunchConfiguration('seed')
    mode = LaunchConfiguration('mode')
    out = LaunchConfiguration('output_dir')
    estop = LaunchConfiguration('publish_estop')
    perception = LaunchConfiguration('perception')
    bus = LaunchConfiguration('bus')
    # Nodes that must acknowledge /scenario/reset (contract §7). M1-B's
    # demand_predictor does not implement the handshake yet, so it is left out.
    participants = PythonExpression([
        "'safety_supervisor,actuator_bridge,metrics_logger,dashboard_marker_publisher'"
        " + (',passenger_detector' if '", perception, "' == 'true' else '')"
        " + (',bus_manager' if '", bus, "' == 'true' else '')"])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S9'),
        DeclareLaunchArgument('seed', default_value='9'),
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('output_dir', default_value='output'),
        DeclareLaunchArgument('publish_estop', default_value='true'),
        DeclareLaunchArgument('perception', default_value='false'),
        DeclareLaunchArgument('record_dashboard', default_value='false'),
        DeclareLaunchArgument('coordination', default_value='true'),
        DeclareLaunchArgument('bus', default_value='true'),
        Node(package='smart_hub', executable='scenario_runner', name='scenario_runner',
             output='screen',
             parameters=[{'scenario': scenario, 'seed': seed, 'publish_estop': estop,
                          'reset_participants': participants}]),
        Node(package='smart_hub', executable='safety_supervisor', name='safety_supervisor',
             output='screen', parameters=[cfg, {'scenario_id': scenario}]),
        Node(package='smart_hub', executable='actuator_bridge', name='actuator_bridge',
             output='screen', parameters=[cfg, {'mode': mode}]),
        Node(package='smart_hub', executable='metrics_logger', name='metrics_logger',
             output='screen',
             parameters=[{'output_dir': out, 'scenario_id': scenario, 'seed': seed}]),
        Node(package='smart_hub', executable='dashboard_marker_publisher',
             name='dashboard_marker_publisher', output='screen'),
        Node(package='smart_hub', executable='dashboard_snapshot', name='dashboard_snapshot',
             output='screen', condition=IfCondition(LaunchConfiguration('record_dashboard')),
             parameters=[{'output_dir': out}]),
        # coordination:=true: crossing arbiter + dispatch (M2-B), parking and
        # traffic state (M3) on the simulator's /hub/vehicles and /parking/observed.
        Node(package='smart_hub', executable='hub_coordinator', name='hub_coordinator',
             output='screen', condition=IfCondition(LaunchConfiguration('coordination')),
             parameters=[{'scenario_id': scenario}]),
        Node(package='smart_hub', executable='parking_manager', name='parking_manager',
             output='screen', condition=IfCondition(LaunchConfiguration('coordination'))),
        Node(package='smart_hub', executable='traffic_monitor', name='traffic_monitor',
             output='screen', condition=IfCondition(LaunchConfiguration('coordination'))),
        # bus:=true: S3 bus docking (M2-A); idle until a DockBus goal arrives.
        Node(package='smart_hub', executable='bus_manager', name='bus_manager',
             output='screen', condition=IfCondition(bus),
             parameters=[{'scenario_id': scenario}]),
        # perception:=true adds the S1/S2 chain on the simulator camera:
        # passenger_detector (M1-A) -> /hub/queue -> demand_predictor (M1-B).
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             output='screen', condition=IfCondition(perception),
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'scenario_id': scenario}]),
        Node(package='smart_hub', executable='demand_predictor', name='demand_predictor',
             output='screen', condition=IfCondition(perception),
             parameters=[os.path.join(share, 'config', 'demand_predictor.yaml'),
                         {'scenario_id': scenario}]),
    ])
