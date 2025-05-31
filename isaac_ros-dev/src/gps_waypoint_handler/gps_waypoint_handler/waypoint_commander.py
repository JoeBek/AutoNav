from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GPSCommander:
    def gps_commander_bringup(self):
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

        # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

        navigator.followWaypoints(goal_poses)
        while not navigator.isTaskComplete():
            print('Task not Completed Yet!')

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
