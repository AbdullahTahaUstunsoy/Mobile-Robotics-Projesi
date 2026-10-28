from setuptools import find_packages, setup

package_name = "behavior"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mission.yaml']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='WASD teleop, camera color detection, lidar obstacle detection, bumper safety and greenhouse mission nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wasd_teleop = behavior.wasd_teleop_node:main',
            'color_detector = behavior.color_detector_node:main',
            'obstacle_detector = behavior.obstacle_detector_node:main',
            'safety = behavior.safety_node:main',
            'greenhouse_mission = behavior.mission_node:main',
        ],
    },
)
