from setuptools import setup

package_name = 'wcm6800_sensor'

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
    description='wcm6800_sensor package for anomaly_sensor_ros2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wcm6800_node = wcm6800_sensor.wcm6800_uart_node:main',
        ],
    },
)
