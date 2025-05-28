from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Extract the cartesian coordinates for each GPS waypoint (Calculated in gps_conversions.py)
    waypoint_positions = []
    with open("/autonav/isaac_ros-dev/src/gps_waypoint_handler/gps_waypoint_handler/stored_waypoints.txt") as waypoint_file:
       for cur_line in waypoint_file:
            x_str, y_str = cur_line.strip().split(',')
            waypoint_positions.append((float(x_str), float(y_str)))

    # Assign each converted GPS waypoint to Goal Poses in the current map frame
    goal_poses = []
    for cur_waypoint_x_pos, cur_waypoint_y_pos in waypoint_positions:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = cur_waypoint_x_pos
        goal_pose.pose.position.y = cur_waypoint_y_pos
        goal_pose.pose.orientation.w = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_poses.append(goal_pose)

    navigator.followWaypoints(goal_poses)
    while not navigator.isTaskComplete():
        current_waypoint = navigator.getFeedback().current_waypoint + 1
        print(f"Currently Navigating to Goal {current_waypoint}")

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('All Goals Navigated!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown() # Shuts down all of NAV2's BT Behavior
    exit(0)

if __name__ == '__main__':
    main()
