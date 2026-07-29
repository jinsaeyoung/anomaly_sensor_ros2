from setuptools import setup

package_name = 'respeaker'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='respeaker package for anomaly_sensor_ros2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker.respeaker_node:main',
            'respeaker_full_node = respeaker.respeaker_full_node:main',
        ],
    },
)
