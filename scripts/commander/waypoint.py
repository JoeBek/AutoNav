from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped



def main():
    # Initialize the ROS client library
    rclpy.init()
    
    # Create a navigator instance
    navigator = BasicNavigator()
    
    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()
    
    # Define waypoints
    waypoints = []
    
    # Create pose 1
    pose1 = PoseStamped()
    pose1.header.frame_id = 'map'
    pose1.pose.position.x = 5.0
    pose1.pose.position.y = 0.0
    waypoints.append(pose1)
    
    # Create pose 2
    pose2 = PoseStamped()
    pose2.header.frame_id = 'map'
    pose2.pose.position.x = 5.0
    pose2.pose.position.y = -5.0
    waypoints.append(pose2)
    
    # Follow waypoints
    navigator.followWaypoints(waypoints)
    
    # Wait for result
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f'Current waypoint: {feedback.current_waypoint}')
        rclpy.spin_once(navigator)
        
    result = navigator.getResult()
    if result:
        print('Goal succeeded!')
    else:
        print('Goal failed!')
        
    rclpy.shutdown()
