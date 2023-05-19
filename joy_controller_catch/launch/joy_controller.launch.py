from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_controller = Node(
        package='joy_controller_catch',
        executable='joy_controller_catch',
    )
    joy_linux = Node(
        package='joy_linux',
        executable='joy_linux_node',
    )
    return LaunchDescription([joy_controller, joy_linux])
