import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from .gps_conversions import GPSHandler
from .waypoint_commander import GPSCommander
import time

class GPSWaypointBringup(Node):

    def __init__(self):
        super().__init__('gps_waypoint_bringup')
        self.subscription = self.create_subscription(
            Bool,
            'autonomous_mode',
            self.listener_callback,
            10)
        self.subscription
        self.autonomous_mode_checker = False

    def listener_callback(self, msg: Bool):
        cur_mode = msg.data # True -> Autonomous Mode / False -> Manual Mode
        if (cur_mode) and (not self.autonomous_mode_checker):
            print("Within Autonomous Mode")
            self.autonomous_mode_checker = True
            # <-------------------------------------- CHANGE DURING COMPETITION -------------------------------------->
            # Populate the GPSHandler Object with the following:
                # Current Latitude of stationary robot position -> Degrees
                # Current Longitude of stationary robot position -> Degrees
                # Current Heading of robot in degrees with respect to Magnetic North -> Degrees
                # Current Magnetic Declination for robot's location (look this up) -> Degrees
            gps_waypoint_handler = GPSHandler(37.23112838066563, -80.42459097426263, -134.8, 0.0)

            # <-------------------------------------- CHANGE DURING COMPETITION -------------------------------------->
            # Populate this list with the current GPS Target Waypoints (Latitude, Longitude)
            targets = [
                (37.23112838066563, -80.4248),
                # (40.749000, -73.986000),  # Target 2
                # (40.747500, -73.984500),  # Target 3
            ]
            gps_waypoint_handler.target_waypoints_in_lat_long = targets
            gps_waypoint_handler.waypoint_conversions_bringup() # Populate the text file with waypoints -> goal pose coordinates            

            #time.sleep(0.05)

            gps_commander_handler = GPSCommander()
            gps_commander_handler.gps_commander_bringup()
        else:
            if (not cur_mode):
                print("Outside of Autonomous Mode")
                self.autonomous_mode_checker = False

def main(args=None):
    rclpy.init(args=args)
    gps_waypoint_bringup = GPSWaypointBringup()
    rclpy.spin(gps_waypoint_bringup)
    gps_waypoint_bringup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()