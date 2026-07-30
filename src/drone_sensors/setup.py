from setuptools import setup
import os
from glob import glob

package_name = 'drone_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Integrated launch package for anomaly_sensor_ros2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_record_node = drone_sensors.auto_record_node:main',
        ],
    },
)
