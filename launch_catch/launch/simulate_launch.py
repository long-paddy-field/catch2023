from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('monitor_catch'),
        "urdf", "robot_arm.urdf")
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
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
        ),
        Node(
            package="dummy_robot_catch",
            executable="dummy_robot_catch",
            name="dummy_robot_catch",
        ),
        Node(
            package="auto_cmd_generator",
            executable="auto_cmd_generator",
            name="auto_cmd_generator",
        ),
        Node(
            package="launch_catch",
            executable="launch_catch",
            name="launch_catch",
            parameters=[{
                os.path.join(
                    get_package_share_directory("launch_catch"),
                    "config",
                    "config.yaml",
                )}],
        ),
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
    ])
