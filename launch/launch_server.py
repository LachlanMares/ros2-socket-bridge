from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_socket_bridge',
            executable='multi_message_server',
            name='multi_message_server',
            remappings=[
                ('/uint8', '/server/uint8'),
                ('/float32', '/server/float32'),
            ]
        ),
        Node(
            package='ros2_socket_bridge',
            executable='multi_message_publisher',
            name='multi_message_publisher',
        )
    ])