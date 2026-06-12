from setuptools import setup

package_name = 'thl100_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'thl100_node = thl100_sensor.thl100_uart_node:main',
        ],
    },
)
