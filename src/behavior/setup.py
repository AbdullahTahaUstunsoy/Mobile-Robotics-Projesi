from setuptools import find_packages, setup

package_name = 'behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='WASD teleop, camera color detection, lidar obstacle detection and bumper safety nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wasd_teleop = behavior.wasd_teleop_node:main',
            'color_detector = behavior.color_detector_node:main',
            'obstacle_detector = behavior.obstacle_detector_node:main',
            'safety = behavior.safety_node:main',
            'imu_monitor = behavior.imu_monitor_node:main',
            'navigation = behavior.navigation_node:main',
            'turn_90 = behavior.turn_90_node:main',
            'l_route = behavior.l_route_node:main',
            'waypoint = behavior.waypoint_node:main',
            'multi_waypoint = behavior.multi_waypoint_node:main',
            'greenhouse_route = behavior.greenhouse_route_node:main',
            'aisle_transition = behavior.aisle_transition_node:main',
        ],
    },
)
