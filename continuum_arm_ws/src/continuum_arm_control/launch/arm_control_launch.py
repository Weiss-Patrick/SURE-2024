import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js2',  # Adjust this to your joystick device
            'deadzone': 0.15
        }]
    )

    arm_controller_node = Node(
        package='continuum_arm_control',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        arm_controller_node
    ])
