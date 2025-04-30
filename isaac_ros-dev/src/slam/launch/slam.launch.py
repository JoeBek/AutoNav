import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


'''
Launch file for slam launch. This version is not GPU dependent.

This system creates map and performs all localization, and publishes map->odom->base_link.

The transforms published are the requirements for the rest of NAV2 to work properly. 
'''

def generate_launch_description():
        # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true')

    pointcloud_topic = DeclareLaunchArgument( 
                            'pointcloud_topic',
                            default_value='/depth_camera/points',
                            description='cloud topic to use for cloud 2 scan' )
    
    publish_period = DeclareLaunchArgument(
        'publish_period',
        # 0.02 if you want to publish
        default_value='0.02',
        description="if you want SLAM to publish map->odom... (sim yes real no)"
    )
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Path to your custom Nav2 parameters file'
    )


    
    # rest in peace ... i will eternalize it in these comments
    # magic_spell = lambda x : 0.02 if x else 0.00
                                            

    pkg_share = FindPackageShare(package='slam').find('slam')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local_sim.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
        
        # 1. LiDAR PointCloud to LaserScan Conversion

        # deps for Lidar 2 pointcloud (remove)
        #sudo apt update
        #sudo apt install ros-humble-vision-msgs ros-humble-tf2-sensor-msgs
        # sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs


        # !!!
    point2laser = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc2_to_laserscan',
        parameters=[{
            'min_height': 0.1,
            'max_height': 0.5,
            'angle_increment': 0.0045,
            'range_max': 30.0,
            'use_inf': True,
            'target_frame': 'base_link',
            'transform_tolerance': 0.03
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('pointcloud_topic')),
            ('scan', '/scan')
        ]
    )

    # 2. SLAM Toolbox (Online Async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {"use_sim_time": LaunchConfiguration('use_sim_time'),
                                  "transform_publish_period": LaunchConfiguration('publish_period')
                                  }]
    )
   
    # 2. Include the Nav2 bringup launch file with your params
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'), 
            # 'params_file': LaunchConfiguration('nav2_params')
        }.items()
    )
    
    '''
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    '''

    # 3. Local EKF (odom -> base_link)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node', # this has to be called ekf_node or slam toolbox freaks the fuck out
        output='screen',
        parameters=[ekf_local_config , {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', 'local_ekf/odom')]
    )

    # 4. GPS Transformation Node
    gps_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            {'zero_altitude': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', 'global_ekf/odom')
        ]
    )


    # 5. Global EKF (map -> odom)
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_config , {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('odometry/filtered', 'global_ekf/odom')
        ]
    )

    return LaunchDescription([
        # params
        publish_period,
        pointcloud_topic,
        use_sim_time,
        nav2_params,
        #nodes
        point2laser,
        ekf_local,
        slam_toolbox,
       # gps_transform,
        #ekf_global, 
        #nav2
        
    ])

