from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import rclpy


def create_pose(pos, quat, frame='odom'):
    new_pose = PoseStamped()
    new_pose.header.frame_id = 'odom'
    px,py,pz = pos
    qx,qy,qz,qw = quat
    new_pose.pose.position.x = px  
    new_pose.pose.position.y = py  
    new_pose.pose.position.z = pz
    new_pose.pose.orientation.x = qx 
    new_pose.pose.orientation.y = qy
    new_pose.pose.orientation.z = qz
    new_pose.pose.orientation.w = qw 
    return new_pose 

def create_path(start_xy, goal_xy, num_points=10, frame='odom'):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = rclpy.clock.Clock().now().to_msg()

    # Create linear interpolated path
    for i in range(num_points):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = start_xy[0] + (goal_xy[0]-start_xy[0])*i/(num_points-1)
        pose.pose.position.y = start_xy[1] + (goal_xy[1]-start_xy[1])*i/(num_points-1)
        pose.pose.orientation.w = 1.0  # Maintain heading
        path.poses.append(pose)
    return path


def main():
        
    rclpy.init()
    nav = BasicNavigator()

    init_pos = (0.0,0.0,0.0)
    init_quat = (0.0,0.0,0.0,1.0)

    goal_pos = (2.0,0.0,0.0)
    goal_quat = (0.0,0.0,0.0,1.0)

    init_pose = create_pose(init_pos, init_quat)
    goal_pose = create_pose(goal_pos, goal_quat)


    # Set initial pose and wait for Nav2 to become active
    nav.setInitialPose(init_pose)

    # Create a straight line path from current position to goal
    #path = nav.getPath(init_pose, goal_pose)
        # Create a straight-line path manually (bypass global planner)
    path = [init_pose.pose, goal_pose.pose]  # Simple two-point path
    custom_path = create_path((0.0, 0.0), (10.0, 0.0), frame='odom')
    # Send directly to controller
    nav.followPath(custom_path)  # Wrap as PoseStamped array



    # Wait for navigation to complete
    while not nav.isTaskComplete():
        # Process feedback if needed
        feedback = nav.getFeedback()
        
        
    # Get result
    result = nav.getResult()


if __name__ == "__main__":
    main()