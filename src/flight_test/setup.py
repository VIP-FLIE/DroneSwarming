# Copyright 2026 DroneSwarming maintainers.
"""Install the flight_test ROS package and its command line entry points."""

from setuptools import find_packages, setup

package_name = 'flight_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DroneSwarming maintainers',
    maintainer_email='maintainer@example.com',
    description='PX4 SITL flight scenarios and diagnostic assertions.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_test_suite = flight_test.orchestrator:main',
            'flight_test_case = flight_test.runner:main',
        ],
    },
)
