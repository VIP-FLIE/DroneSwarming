"""Install the planner, file-submission CLI, examples, and source attribution."""

from glob import glob
from setuptools import find_packages, setup

setup(
    name='mission_planner',
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mission_planner']),
        ('share/mission_planner', ['package.xml', 'LICENSE.sar-drone']),
        ('share/mission_planner/examples', glob('examples/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DroneSwarming maintainers',
    maintainer_email='maintainer@example.com',
    description='Compile high-level JSON flight intent for one vehicle or a fleet.',
    license='GPL-3.0-only AND BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner_node = mission_planner.node:main',
            'submit_flight_plan = mission_planner.cli:main',
        ]
    },
)
