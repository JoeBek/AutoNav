import os
 
from ament_index_python.packages import get_package_share_directory
 
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
 
from launch_ros.actions import Node
 
 
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='sim' #<--- CHANGE ME

    DeclareLaunchArgument('world,',
                          default_value='src/sim/worlds/autonav_igvc_course.world',
                          description='path to world file')


    pkg_share = FindPackageShare(package='sim').find('sim')
    default_model_path = os.path.join(pkg_share, 'description', 'custom_robot', 'robot.urdf.xacro')
 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
 
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot', '-z 0.5', '-Y -1.570'],
                        output='screen')
   
 
    return LaunchDescription([
        rsp,
        jsp,
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')], output='screen'),
        spawn_entity,
    ])