from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='core_catch',
            executable='core_catch',
            name='core_catch',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy'
        ),
        Node(
            package='joy_controller_catch',
            executable='joy_controller_catch',
            name='joy_controller',
        ),
        Node(
            package='dummy_robot_catch',
            executable='dummy_robot_catch',
            name='dummy_robot',
        )
    ])
