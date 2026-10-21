from glob import glob

from setuptools import find_packages, setup

package_name = 'smart_hub'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages, not [package_name]: sub-packages such as M1-B's
    # smart_hub.demand must be installed too. scripts/ is offline tooling.
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
        ('share/' + package_name + '/models', ['models/demand_ridge_h30.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tuymurod (M1-B), Saidumar (M5)',
    maintainer_email='tuymurodqodirov123456789@gmail.com, egamberdiyevsaidumar@gmail.com',
    description='Smart Mobility Hub application nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # M1-A (passenger detection, S1)
            'passenger_detector = smart_hub.passenger_detector:main',
            'video_publisher = smart_hub.video_publisher:main',
            's1_check = smart_hub.s1_check:main',
            'video_eval = smart_hub.video_eval:main',
            # M2-B (hub coordinator, S8)
            'hub_coordinator = smart_hub.hub_coordinator:main',
            # M3 (parking, S4; traffic, S5)
            'parking_manager = smart_hub.parking_manager:main',
            'traffic_monitor = smart_hub.traffic_monitor:main',
            # M1-B (AI demand prediction, S2)
            'demand_predictor = smart_hub.demand_predictor:main',
            'fake_queue_publisher = smart_hub.fake_queue_publisher:main',
            # M5 (simulator, safety, infrastructure, S9)
            'scenario_runner = smart_hub.scenario_runner:main',
            'safety_supervisor = smart_hub.safety_supervisor:main',
            'actuator_bridge = smart_hub.actuator_bridge:main',
            'metrics_logger = smart_hub.metrics_logger:main',
            'dashboard_marker_publisher = smart_hub.dashboard_marker_publisher:main',
            'dashboard_snapshot = smart_hub.dashboard_snapshot:main',
            # M5 test tools (never part of the integrated hub launch)
            'mock_candidate_publisher = smart_hub.mock_candidate_publisher:main',
            's9_fault_check = smart_hub.s9_fault_check:main',
            's9_repeat_check = smart_hub.s9_repeat_check:main',
        ],
    },
)
