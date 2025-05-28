import rclpy
import serial
import time
from serial import Serial
from rclpy.node import Node
from std_msgs.msg import String  


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('estop_driver')
        self.publisher_ = self.create_publisher(String, 'estop', 10)  # Message type, topic name, queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        baudrate = 9600 
        self.cereal = Serial('/dev/ttyTHS1', baudrate, timeout=3)
        self.cereal.stopbits = 1
        self.cereal.parity = 'N'
        self.cereal.bytesize = 8

        self.encoding = 'ascii'
        self.get_logger().info('This is an info message!')



    def timer_callback(self):
        msg = String()
        while self.cereal.in_waiting <= 0:
            pass
        line = self.cereal.readline().decode(self.encoding)
        msg.data = f'{line}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

