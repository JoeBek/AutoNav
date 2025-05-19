import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction
)

from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown
)


# joy | core | lidar
#  |
#  v
#control  
#  | 
#  v
# odom ---------> SLAM 


def generate_launch_description():

    # joy | core | 

    sick_pkg = os.path.join(get_package_share_directory('sick_scan_xd'), 'launch', 'sick_multiscan.launch.py')
    sick_pkg = PathJoinSubstitution([FindPackageShare('sick_scan_xd'), 'launch', 'sick_multiscan.launch.py'])
    core_pkg = PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'core_bringup.launch.py'])
    control_pkg = PathJoinSubstitution([FindPackageShare('control'), 'launch', 'control_dev.launch.py'])
    
    

    sick_args = {
        'hostname':'192.168.0.1',
        'udp_receiver_ip':'192.168.0.2',
        'publish_frame_id':'lidar_footprint',
        'tf_publish_rate':'0'
    }


    sick = IncludeLaunchDescription(
             sick_pkg, launch_arguments=sick_args.items()
    )
    core = IncludeLaunchDescription(
            core_pkg ,
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
    )

    control = IncludeLaunchDescription(
        control_pkg
    )

    odom = Node(
        pacakge='odom_handler',
        executable='wheel_odometry_publisher',
        name='wheel_odom'
    )

    control_event = RegisterEventHandler(
        OnProcessIO(
            target_action=joy,
            on_stdout=[
                LogInfo(msg='Control node booting up...'),
                control
            ]
        )
    )
    odom_event = RegisterEventHandler(
        OnProcessIO(
            target_action=control,
            on_stdout=[
                LogInfo(msg='Odom node booting up...'),
                odom
            ]
        )
    )


    return LaunchDescription([
        joy,
        sick,
        core,
        control_event,
        odom_event
       
    ])

    
    

