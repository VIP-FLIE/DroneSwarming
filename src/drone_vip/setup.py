from setuptools import find_packages, setup

package_name = 'drone_vip'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UD Drone VIP',
    maintainer_email='dmihovch@udel.edu',
    description='Starter nodes for the UD Drone VIP swarm project.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'listener = drone_vip.listener:main',
            'fly = drone_vip.takeoff_and_fly:main',
        ],
    },
)
