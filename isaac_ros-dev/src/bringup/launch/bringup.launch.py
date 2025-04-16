
import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro
 
from launch_ros.actions import Node
 
'''
Launches entire stack.
SLAM launch can be found in SLAM package... might move it here

core launch -> TF
SLAM launch -> localization nodes and SLAM (map->odom->base_link)
NAV2 launch -> pathing, costmap
'''

def generate_launch_description():
    
    slam_pkg_share = FindPackageShare('slam')
    pkg_share = FindPackageShare('bringup')
    
    slam_pkg = PathJoinSubstitution([slam_pkg_share, 'launch', 'slam.launch.py'])
    bringup_pkg = PathJoinSubstitution([pkg_share, 'launch', 'core_bringup.launch.py'])
    
    bowser = IncludeLaunchDescription( PythonLaunchDescriptionSource (bringup_pkg) )
    slam = IncludeLaunchDescription( PythonLaunchDescriptionSource (slam_pkg) )
    
    
    
