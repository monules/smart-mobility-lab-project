from glob import glob

from setuptools import find_packages, setup

package_name = 'smart_hub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/demand_ridge_h30.json']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tuymurod',
    maintainer_email='tuymurodqodirov123456789@gmail.com',
    description='Smart Mobility Hub - S2 AI demand prediction (M1-B)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demand_predictor = smart_hub.demand_predictor:main',
            'fake_queue_publisher = smart_hub.fake_queue_publisher:main',
        ],
    },
)
