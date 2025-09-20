from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_driver',
            executable='gps_node',
            name='gps_node'
        )
    ])