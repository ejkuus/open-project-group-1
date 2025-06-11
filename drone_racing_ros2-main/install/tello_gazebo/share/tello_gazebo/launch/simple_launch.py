"""Simulate a Tello drone with Gazebo"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Launch Gazebo server and client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Spawn tello
        Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            arguments=[urdf_path, '0', '0', '1', '1.57079632679'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),

        # Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            namespace=ns,
            output='screen'
        ),

        # Joystick control for Tello
        Node(
            package='tello_driver',
            executable='tello_joy_main',
            namespace=ns,
            output='screen'
        ),
    ])
