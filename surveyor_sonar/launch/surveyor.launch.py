from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='surveyor_sonar',
            executable='surveyor_node',
            name='surveyor_node',
            parameters=[
                {"ip": "192.168.2.86"},
                {"port": 62312}
            ]
        )
    ])