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
    description='WASD keyboard teleop that publishes motion commands on /cmd_vel.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wasd_teleop = behavior.wasd_teleop_node:main',
            'color_detector = behavior.color_detector_node:main',
        ],
    },
)
