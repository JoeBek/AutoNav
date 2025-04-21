
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription



def generate_launch_description():

    zed_wrapper_share = FindPackageShare('zed_wrapper')
    sick_scan_share = FindPackageShare('sick_scan_xd')


    # ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.0.1 udp_receiver_ip:=255.255.255.0


    zed_pkg = PathJoinSubstitution([zed_wrapper_share, 'launch', 'zed_wrapper.launch.py'])
    sick_pkg = PathJoinSubstitution([sick_scan_share, 'launch', 'sick_multiscan.launch.py'])
    

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (zed_pkg), launch_arguments={'camera_model':'2i'}.items()
    )

    sick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource (sick_pkg), launch_arguments={'hostname':'192.168.0.1', 'udp_receiver_ip':'255.255.255.0'}.items()
    )


    return LaunchDescription([
        zed_pkg,
        sick_pkg
    ])

    
    