from setuptools import setup

package_name = 'wcm6800_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'wcm6800_node = wcm6800_sensor.wcm6800_uart_node:main',
        ],
    },
)
