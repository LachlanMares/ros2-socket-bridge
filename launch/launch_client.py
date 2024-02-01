from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_socket_bridge',
            executable='multi_message_client',
            name='multi_message_client',
            remappings=[
                ('/string', '/client/string'),
                ('/odometry', '/client/odometry'),
                ('/pose', '/client/pose'),
                ('/imu', '/client/imu'),
            ]
        ),
        Node(
            package='ros2_socket_bridge',
            executable='multi_message_subscriber',
            name='multi_message_subscriber',
        )
    ])