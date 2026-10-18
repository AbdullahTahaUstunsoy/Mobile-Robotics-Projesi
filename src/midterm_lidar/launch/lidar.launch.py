"""Start the lidar obstacle detector and an RViz view of the scan and obstacles.

Run it next to the simulation (without its own RViz window):
    ros2 launch midterm_bringup sim.launch.py rviz:=false
    ros2 launch midterm_lidar lidar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('midterm_lidar')
    rviz = LaunchConfiguration('rviz')

    obstacle_detector = Node(
        package='midterm_lidar',
        executable='obstacle_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(lidar_share, 'rviz', 'lidar.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        obstacle_detector,
        rviz_node,
    ])
