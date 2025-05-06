from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

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



    controller = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/path/to/nav2_params.yaml', {"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )
    costmap = Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_ros',
            name='local_costmap',
            output='screen',
            parameters=['/path/to/nav2_params.yaml', {"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )
    navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['/path/to/nav2_params.yaml', {"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )
       
    return LaunchDescription([
        use_sim_time,
        nav2_params,
        controller,
        costmap,
        navigator
    ])