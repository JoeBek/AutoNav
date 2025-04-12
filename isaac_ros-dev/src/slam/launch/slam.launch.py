from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

'''
Launch file for slam launch. This version is not GPU dependent.

This system creates map and performs all localization, and publishes map->odom->base_link.

The transforms published are the requirements for the rest of NAV2 to work properly. 
'''

def generate_launch_description():
        # Launch Arguments
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        # 1. LiDAR PointCloud to LaserScan Conversion
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
            ('cloud_in', '/multiscan100/points'),
            ('scan', '/scan')
        ]
    ),

    # 2. SLAM Toolbox (Online Async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_pkg'),
                'config',
                'slam_config.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    ),

    # 3. Local EKF (odom -> base_link)
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_pkg'),
                'config',
                'local_ekf.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', 'local_ekf/odom')
        ]
    ),

    # 4. Global EKF (map -> odom)
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_pkg'),
                'config',
                'global_ekf.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', 'global_ekf/odom')
        ]
    ),

    # 5. GPS Transformation Node
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

    return LaunchDescription([
        point2laser,
        slam_toolbox,
        local_ekf,
        gps_transform,
        global_ekf
        
    ])