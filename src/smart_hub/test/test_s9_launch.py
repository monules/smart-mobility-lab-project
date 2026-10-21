"""launch_testing: run the full S9 sequence on the M5 launch and require a pass.

Starts hub.launch.py (scenario_runner, safety_supervisor, actuator_bridge,
metrics_logger) plus s9_fault_check, waits for the check to finish, and
asserts it exited 0 with every check (T16, T17 controller loss, reset rules)
passing in its JSON evidence.
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

OUT = tempfile.mkdtemp(prefix='s9_launch_')


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('smart_hub')
    check = Node(package='smart_hub', executable='s9_fault_check', name='s9_fault_check',
                 parameters=[{'output_dir': OUT, 't16_trials': 3}], output='screen')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '88'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S9', 'seed': '9', 'output_dir': OUT,
                              'publish_estop': 'false',
                              'robot': 'false'}.items()),   # the check drives robot_1
        check,
        launch_testing.actions.ReadyToTest(),
    ]), {'check': check}


class TestS9Runs(unittest.TestCase):
    def test_check_finishes(self, proc_info, check):
        proc_info.assertWaitForShutdown(process=check, timeout=150)


@launch_testing.post_shutdown_test()
class TestS9Result(unittest.TestCase):
    def test_check_exit_code(self, proc_info, check):
        launch_testing.asserts.assertExitCodes(proc_info, process=check)

    def test_every_check_passed(self):
        with open(os.path.join(OUT, 's9_check.json')) as fh:
            result = json.load(fh)
        failed = [c for c in result['checks'] if not c['passed']]
        self.assertEqual(failed, [])
        names = {c['check'] for c in result['checks']}
        for required in ('2 T16 stop latency', '4 e-stop false keeps latch',
                         '6 reset rejected: stale odometry', '7 reset accepted when safe',
                         '10 T17 controller loss -> zero'):
            self.assertIn(required, names)
        self.assertTrue(all(x is not None and x <= 0.20
                            for x in result['t16']['external_latency_s']))
