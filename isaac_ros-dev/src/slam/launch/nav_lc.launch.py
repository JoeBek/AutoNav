from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = True  
    autostart = True     

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    return LaunchDescription([
        lifecycle_manager_node
    ])