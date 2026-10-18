from glob import glob

from setuptools import find_packages, setup

package_name = 'midterm_lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='2D lidar obstacle detection with RViz markers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_detector = midterm_lidar.obstacle_detector:main',
        ],
    },
)
