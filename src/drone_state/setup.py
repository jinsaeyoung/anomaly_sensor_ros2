from setuptools import setup

package_name = 'drone_state'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'drone_state_node = drone_state.drone_state_node:main',
        ],
    },
)

