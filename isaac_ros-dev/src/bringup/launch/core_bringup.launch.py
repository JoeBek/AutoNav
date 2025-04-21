
import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
import xacro
 
from launch_ros.actions import Node
 
 
'''
Bowser core launch. Handles all things TF

'''

def generate_launch_description():
 
    pkg_name = 'bringup'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    model_path = os.path.join(get_package_share_directory(pkg_share), 'description', 'bowser.urdf.xacro')
    bowser_description_config = xacro.process_file(model_path)
   # Create a robot_state_publisher node

    params = {'robot_description': bowser_description_config.toxml()}
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[model_path]
    )
    
    return LaunchDescription(
        rsp,
        jsp
    )