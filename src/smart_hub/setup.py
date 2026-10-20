from glob import glob

from setuptools import find_packages, setup

package_name = 'smart_hub'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages so sub-packages (e.g. M1-B's smart_hub.demand) install too
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saidumar (M5)',
    maintainer_email='egamberdiyevsaidumar@gmail.com',
    description='Smart Mobility Hub application nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # M5 (simulator, safety, infrastructure)
            'scenario_runner = smart_hub.scenario_runner:main',
            'safety_supervisor = smart_hub.safety_supervisor:main',
            'actuator_bridge = smart_hub.actuator_bridge:main',
            'metrics_logger = smart_hub.metrics_logger:main',
            'dashboard_marker_publisher = smart_hub.dashboard_marker_publisher:main',
            # M5 test tools (never part of the integrated hub launch)
            'mock_candidate_publisher = smart_hub.mock_candidate_publisher:main',
            's9_fault_check = smart_hub.s9_fault_check:main',
        ],
    },
)
