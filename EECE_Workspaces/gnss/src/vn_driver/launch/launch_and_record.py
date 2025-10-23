from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import time
today = time.localtime()
date_str = f'{today.tm_mon}_{today.tm_mday}_{today.tm_year}_{today.tm_hour}{today.tm_min}{today.tm_sec}'

def generate_launch_description():
    port_address = LaunchConfiguration('port')
    bag_file = LaunchConfiguration('bagfile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Specifies the port address which to listen'
        ),
        DeclareLaunchArgument(
            'bagfile',
            default_value='imu_bag_' + date_str,
            description='Specifies the bag file to record to'
        ),
        Node(
            package='vn_driver',
            executable='vn_node',
            name='vn_node',
            parameters=[{'port':port_address}]
        ),
        ExecuteProcess(
            cmd=['ros2','bag','record','-o', bag_file,'/imu']
        )
    ])