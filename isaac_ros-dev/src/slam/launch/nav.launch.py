from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

# launches nav2 with custom nodes for testing
# assumes publishers already working

def generate_launch_description():
       

    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true')

    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('slam'),
            'config',
            'nav_minimal.yaml'
        ]),
        description='Path to your custom Nav2 parameters file'
    )

    configured_params = RewrittenYaml(
    source_file=LaunchConfiguration('nav2_params'),
    root_key='',
    param_rewrites={},
    convert_types=True
    )
 
    # controls what nodes are managed by lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        #'planner_server',
        'bt_navigator',
        # 'waypoint_follower',
        #'velocity_smoother'
    ]

    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )




    controller = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params]
    )

    navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params]
    )
       
    return LaunchDescription([
        use_sim_time,
        nav2_params,
        controller,
        navigator,
        manager
        
    ])