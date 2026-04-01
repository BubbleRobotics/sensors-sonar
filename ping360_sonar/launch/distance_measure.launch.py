from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    pingNode = Node(
        package = "ping360_sonar",
        executable = "ping360_node",
        parameters = [
            {"range_max": 10},
            {"angle_sector": 20},
            {"publish_distance": True},
            {"publish_echo": True},
            {"angle_step": 4}
        ],
        output = "screen"
    )

    ld.add_action(pingNode)

    return ld