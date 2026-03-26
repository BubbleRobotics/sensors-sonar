from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('surveyor_sonar'),
                'config',
                'surveyor.yaml',
            ]),
            description='Path to the Surveyor ROS parameters file',
        ),
        Node(
            package='surveyor_sonar',
            executable='surveyor_node',
            name='surveyor_node',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
