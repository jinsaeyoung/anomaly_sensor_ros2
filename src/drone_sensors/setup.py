from setuptools import setup
import os
from glob import glob

package_name = 'drone_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),   # launch 파일 포함
    ],
    entry_points={'console_scripts': []},
)

