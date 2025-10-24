import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

package_name = 'yolobot'
today = time.localtime()
date_str = f'{today.tm_mon}_{today.tm_mday}_{today.tm_year}_{today.tm_hour}{today.tm_min}{today.tm_sec}'

def generate_launch_description():

    return LaunchDescription([

        # ExecuteProcess(
        #     # cmd=['export', 'PYTHONPATH=/home/david-ross/gitRepos/ROS2/YOLO/.yolo_venv/lib/python3.12/site-packages:$PYTHONPATH']
        # ),
        Node(
            package=package_name,
            executable='yolo_viz',
            name='yolo_viz',
            # parameters=[{'port':port_address}]
        ),
    ])
#export PYTHONPATH=/home/david-ross/gitRepos/ROS2/YOLO/.yolo_venv/lib/python3.12/site-packages:$PYTHONPATH