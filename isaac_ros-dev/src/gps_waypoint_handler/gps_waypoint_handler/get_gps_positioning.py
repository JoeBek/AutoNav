import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_fix',
            self.listener_callback,
            10)
        self.subscription
        self.current_lat_long_data = []
        self.data_collection_complete = False

    def listener_callback(self, msg: NavSatFix):
        if len(self.current_lat_long_data) < 50:
            lat = msg.latitude
            long = msg.longitude
            self.current_lat_long_data.append((lat, long))
            if (len(self.current_lat_long_data) % 10 == 0):
                print(f"Gathering Data {len(self.current_lat_long_data)} / 50\n")
        if (not self.data_collection_complete) and (len(self.current_lat_long_data) == 50):
            print(f"Gathering Data {len(self.current_lat_long_data)} / 50\n")
            print(f"Data Collection Complete\n")
            avg_lat = sum(lat for lat, _ in self.current_lat_long_data) / len(self.current_lat_long_data)
            avg_long = sum(long for _, long in self.current_lat_long_data) / len(self.current_lat_long_data)
            
            # Print or return the averages
            print(f"Average Latitude: {avg_lat}")
            print(f"Average Longitude: {avg_long}")
            self.data_collection_complete = True

            # Store the current gps positioning into a txt file for later use
            cur_gps_position_file = open("/autonav/isaac_ros-dev/src/gps_waypoint_handler/gps_waypoint_handler/cur_gps_positon.txt", "w")
            stored_data = str(avg_lat) + "," + str(avg_long) + "\n"
            cur_gps_position_file.write(stored_data)
            cur_gps_position_file.close()

            # Destroy and Stutdown the node after average lat and long are computed
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()
