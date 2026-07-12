"""
Setup configuration for rover_core ROS2 package.

Defines package metadata, dependencies, and console script entry points
for the five rover nodes (navigation, safety, vision, motor_control, telemetry_aggregator).
"""

from setuptools import setup, find_packages

package_name = 'rover_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_bringup.launch.py']),
        ('share/' + package_name + '/config', [
            'config/mediamtx.yml',
            'config/params.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='UCMTC Rover Team',
    maintainer_email='rover@ucmtc.org',
    description='Rover Core - ROS2 Hardware Integration and Telemetry Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node=rover_core.navigation_node:main',
            'safety_node=rover_core.safety_node:main',
            'vision_node=rover_core.vision_node:main',
            'motor_control_node=rover_core.motor_control_node:main',
            'telemetry_aggregator=rover_core.telemetry_aggregator:main',
            'ps5_nfs_teleop=rover_core.ps5_nfs_teleop:main',
            'cmd_vel_to_wheels=rover_core.cmd_vel_to_wheels:main',
            'wheel_cmds_serial_bridge=rover_core.wheel_cmds_serial_bridge:main',
        ],
    },
)
