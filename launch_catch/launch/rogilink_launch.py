from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rogilink2",
                executable="rogilink2",
                name="rogilink",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "config_path": os.path.join(
                            get_package_share_directory("launch_catch"),
                            "config",
                            "rogilink.yaml",
                        )
                    }
                ],
            )
        ]
    )
