from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():

    return LaunchDescription([
        Node(package="joy",
             executable="joy_node",
             name="joy",
             parameters=[{
                 "deadzone": 0.005
             }]),
        Node(
            package="joy_controller_catch",
            executable="joy_controller_catch",
            name="joy_controller",
            parameters=[{
                os.path.join(get_package_share_directory('launch_catch'),
                             'config', 'config.yaml')
            }]
        ),
        Node(
            package="converter_principal",
            executable="converter_principal",
            name="converter",
        ),
        Node(
            package="rogilink2",
            executable="rogilink2",
            name="rogilink2",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "config_path":
                os.path.join(
                    get_package_share_directory("launch_catch"),
                    "config",
                    "rogilink.yaml",
                )
            }, ('__log_level:=debug')],
        )
    ])
