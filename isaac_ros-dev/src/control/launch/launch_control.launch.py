import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
from launch_ros.actions import Node

def generate_launch_description():

    package_name='control'

    control_node = Node (
        package='control',
        executable='control'
    )

    return LaunchDescription([control_node])