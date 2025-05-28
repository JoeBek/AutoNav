import math

class GPSHandler:
    def __init__(self, ref_lat:float, ref_long:float, cur_heading:float, cur_decl:float):
        self.reference_latitude = ref_lat  # The robot's current GPS waypoint latitude on startup
        self.reference_longitude = ref_long  # The robot's current GPS waypoint longitude on startup
        # The robot's heading is the difference in degrees between the robot's current orientation and 
        # MAGNETIC NORTH. For instance if the robot is currently oriented 30 degrees due west with respect
        # to MAGNETIC NORTH then the heading would be -30 degrees. Vise Versa if the robot was oriented
        # 30 degrees due east with respect to MAGNETIC NORTH the heading would be +30 degrees.
        self.current_heading = cur_heading # The robot's current heading with respect to MAGNETIC NORTH
        # Positive Magnetic Declination is subtracted to heading and Negative Magnetic Declination is added to heading 
        self.current_declination = cur_decl # Magnetic Declination of current location (WILL HAVE TO CHANGE ON COMPETITION!!!)
        self.current_waypoints = {} # Dictionary of current waypoints in meters with respect to the reference GPS waypoint
        # Waypoint Dictionary Format:
        # self.current_waypoints = {
        # "Waypoint1": [Distance in X Direction (meters), Distance in Y Direction (meters)] 
        # }

    # Function to calculate distance in x (longitude) and y (latitude) directions
    def calculate_distance(self, target_latitude:float, target_longitude:float):
        # Earth's radius in meters
        radius = 6371000

        # Convert latitudes and longitudes from degrees to radians
        reference_lat_rads = math.radians(self.reference_latitude)
        reference_long_rads = math.radians(self.reference_longitude)
        target_lat_rads = math.radians(target_latitude)
        target_long_rads = math.radians(target_longitude)

        # Difference in latitudes and longitudes
        delta_lat = target_lat_rads - reference_lat_rads
        delta_lon = target_long_rads - reference_long_rads

        # Y direction (latitude) distance in meters
        y_distance = delta_lat * radius

        # X direction (longitude) distance in meters, adjusted for latitude
        x_distance = delta_lon * radius * math.cos(reference_lat_rads)

        return x_distance, y_distance

    # Function to apply the heading offset to the x and y distances
    def apply_heading_offset(self, x, y):
        # Convert heading to radians
        cur_heading_rads = math.radians(self.current_heading)

        # Rotate the x and y distances by the heading angle
        new_x = x * math.cos(cur_heading_rads) - y * math.sin(cur_heading_rads)
        new_y = x * math.sin(cur_heading_rads) + y * math.cos(cur_heading_rads)

        return new_x, new_y

    # Function to handle multiple GPS target waypoints
    def process_multiple_targets(self, target_waypoints):
        cur_num_of_targets = len(self.current_waypoints)
        for cur_target_lat, cur_target_long in target_waypoints:
            # Calculate x and y distances
            waypoint_dict_key = "Waypoint" + str(cur_num_of_targets + 1)
            x, y = self.calculate_distance(cur_target_lat, cur_target_long)
            
            # Apply heading adjustment
            adjusted_x, adjusted_y = self.apply_heading_offset(x, y)

            # The map that NAV2 generates is rotated 90 degrees counterclockwise about our current
	    # robot's orientation so we have to rotate our coordinates 90 degrees clockwise to align
	    # with NAV2's map
            final_x = adjusted_y
            final_y = -adjusted_x

            # Store the adjusted target coordinates
            self.current_waypoints[waypoint_dict_key] = [final_x, final_y]
            cur_num_of_targets += 1        

def main(args=None):
    # <-------------------------------------- CHANGE DURING COMPETITION -------------------------------------->
    # Populate the GPSHandler Object with the following:
        # Current Latitude of stationary robot position -> Degrees
        # Current Longitude of stationary robot position -> Degrees
        # Current Heading of robot in degrees with respect to Magnetic North -> Degrees
        # Current Magnetic Declination for robot's location (look this up) -> Degrees
    gps_waypoint_handler = GPSHandler(37.23141563653532, -80.4243882640172, -141.5, 0.0)

    # <-------------------------------------- CHANGE DURING COMPETITION -------------------------------------->
    # Populate this list with the current GPS Target Waypoints (Latitude, Longitude)
    targets = [
        (37.23127661772232, -80.42445879127482),
        # (40.749000, -73.986000),  # Target 2
        # (40.747500, -73.984500),  # Target 3
    ]

    # Negative Declination means TRUE NORTH IS shifted clockwise by however many degrees specified.
    # For instance in Blacksburg the magnetic declination is around -8 degrees. Meaning if you are 
    # facing Magnectic North provided by your IPhone compass you would turn 8 degrees due East, i.e. 
    # clockwise, to be facing TRUE NORTH. This also means that now referencing that TRUE NORTH orientation
    # you would take the difference in degrees from the TRUE NORTH and the current orientation of the robot.
    gps_waypoint_handler.process_multiple_targets(targets)

    # Write GPS Waypoint Conversions to txt file for later use (MAKE SURE TO CHANGE ABSOLUTE PATH ON JETSON)
    waypoints_file = open("/autonav/isaac_ros-dev/src/gps_waypoint_handler/gps_waypoint_handler/stored_waypoints.txt", "w")
    for waypoint in gps_waypoint_handler.current_waypoints.values():
        lat = waypoint[0]
        long = waypoint[1]
        stored_data = str(lat) + "," + str(long) + "\n"
        print(stored_data)
        waypoints_file.write(stored_data)
    waypoints_file.close()

if __name__ == '__main__':
    main()
