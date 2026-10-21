"""launch_testing: S1 (M1-A) end to end on the real ROS graph.

Simulator camera -> passenger_detector -> /hub/queue, checked by s1_check
against simulator truth: T01 counts 0,3,5,4, T02 occlusion and freeze, and
the 2 s crossing hold. Uses the deterministic blob baseline.
"""
import json
import os
import tempfile
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

OUT = tempfile.mkdtemp(prefix='s1_launch_')


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('smart_hub')
    check = Node(package='smart_hub', executable='s1_check', name='s1_check',
                 parameters=[{'output_dir': OUT, 'exact_counts': True}], output='screen')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '91'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S1', 'seed': '1', 'output_dir': OUT}.items()),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             parameters=[os.path.join(share, 'config', 'passenger_detector.yaml'),
                         {'detector': 'blob'}]),
        check,
        launch_testing.actions.ReadyToTest(),
    ]), {'check': check}


class TestS1Runs(unittest.TestCase):
    def test_check_finishes(self, proc_info, check):
        proc_info.assertWaitForShutdown(process=check, timeout=90)


@launch_testing.post_shutdown_test()
class TestS1Result(unittest.TestCase):
    def test_check_exit_code(self, proc_info, check):
        launch_testing.asserts.assertExitCodes(proc_info, process=check)

    def test_every_check_passed(self):
        with open(os.path.join(OUT, 's1_check.json')) as fh:
            result = json.load(fh)
        self.assertEqual([c for c in result['checks'] if not c['passed']], [])
        names = {c['check'] for c in result['checks']}
        for required in ('T01 stable counts', 'T02 no double count',
                         'T02 freeze -> invalid', 'S1 crossing hold'):
            self.assertIn(required, names)
