"""Start the greenhouse world, spawn the robot, bridge topics, start the color and obstacle detectors
and the safety filter, and open RViz.

Drive it with WASD teleop in a second terminal:
    ros2 run behavior wasd_teleop
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    description_share = get_package_share_directory('description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    xacro_file = os.path.join(description_share, 'urdf', 'greenhouse_bot.urdf.xacro')
    # Process xacro in Python (a shell 'xacro <path>' command breaks on paths with spaces)
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        # gz_sim.launch.py runs gz through a shell: quote the world path (it may contain spaces)
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-s --headless-rendering ' if '", headless, "' == 'true' else ''"]),
                '-r "', world, '"',
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'greenhouse_bot',
            '-x', x, '-y', y, '-z', '0.05', '-Y', yaw,
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    color_detector = Node(
        package='behavior',
        executable='color_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    obstacle_detector = Node(
        package='behavior',
        executable='obstacle_detector',
        parameters=[
            os.path.join(bringup_share, 'config', 'obstacle_detector.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # Teleop / navigation publish /cmd_vel_raw; safety blocks forward motion on bumper contact or
    # lidar STOP and drives the robot on /cmd_vel
    safety = Node(
        package='behavior',
        executable='safety',
        parameters=[
            os.path.join(bringup_share, 'config', 'safety.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_share, 'rviz', 'greenhouse_bot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup_share, 'worlds', 'greenhouse.sdf'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Gazebo server only (no GUI)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        # Start in aisle 1, heading +x, with the middle row on the right
        DeclareLaunchArgument('x', default_value='-0.5'),
        DeclareLaunchArgument('y', default_value='0.65'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        color_detector,
        obstacle_detector,
        safety,
        rviz_node,
    ])
