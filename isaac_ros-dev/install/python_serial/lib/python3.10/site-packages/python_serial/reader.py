import rclpy
from rclpy.node import Node
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(f'Received: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

