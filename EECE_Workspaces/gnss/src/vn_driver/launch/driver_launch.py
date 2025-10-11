from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    port_address = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Specifies the port address which to listen'
        ),
        Node(
            package='vn_driver',
            executable='vn_node',
            name='vn_node',
            parameters=[{'port':port_address}]
        ),
    ])