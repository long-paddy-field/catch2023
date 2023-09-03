import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('monitor_catch'),
        "urdf", "robot_arm.urdf")
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='monitor_catch',
            executable='monitor_catch',
            name='monitor_catch',
            output='screen'),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('monitor_catch'), 'rviz2', 'robot_arm.rviz')]
        ),
        Node(package="joy", executable="joy_node", name="joy"),
        Node(
            package="joy_controller_catch",
            executable="joy_controller_catch",
            name="joy_controller",
        ),
        Node(
            package="dummy_robot_catch",
            executable="dummy_robot_catch",
            name="dummy_robot_catch",
        )
    ])
