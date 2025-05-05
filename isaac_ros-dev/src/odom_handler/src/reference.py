import serial
import time
import math
import re

ser = serial.Serial('COM7', 115200, timeout=0.1)

class Odom():
    def __init__(self):
        self.delta_left = 0
        self.prev_left_encoder_count = 0
        self.left_displacement = 0.0

        self.delta_right = 0
        self.prev_right_encoder_count = 0
        self.right_displacement = 0.0

        self.distance_between_wheels = 0.6858
        self.wheel_radius = 0.205
        self.encoder_counts_per_revolution = 81923

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def get_wheel_displacement(self, encoder_count):
        wheel_displacement = (2 * math.pi * self.wheel_radius) * (encoder_count / self.encoder_counts_per_revolution)
        #Case for gear ratio included
            #wheel_displacement = (2 * math.pi * self.wheel_radius) * (self.gear_ratio * encoder_count / self.encoder_counts_per_revolution)
        return wheel_displacement

    def update_odom(self, cur_left_encoder_count, cur_right_encoder_count):
        #Get the difference in encoder count since last update
        self.delta_left = cur_left_encoder_count - self.prev_left_encoder_count
        self.delta_right = cur_right_encoder_count - self.prev_right_encoder_count

        #Update to the most recent encoder count reading
        self.prev_left_encoder_count = cur_left_encoder_count
        self.prev_right_encoder_count = cur_right_encoder_count

        #Calculate the wheel displacement for each wheel
        self.left_displacement = self.get_wheel_displacement(self.delta_left)
        #print(self.left_displacement)
        self.right_displacement = self.get_wheel_displacement(self.delta_right)
        #print(self.right_displacement)

        #Calculate the distance by reference point (in-between the two wheels) and change in angle
        distance_traveled = (self.left_displacement + self.right_displacement) / 2
        #print(distance_traveled)
        delta_theta = (self.right_displacement - self.left_displacement) / self.distance_between_wheels
        #print(delta_theta)

        #Update odometry in cartesian coordinates
        self.x += (distance_traveled * math.cos(self.theta))  
        #self.x += (distance_traveled * math.cos(delta_theta))      
        #For smooth curved motion    
        #self.x = self.x + (distance_traveled * math.cos(self.theta + (delta_theta / 2)))      
        print(f"X-Coordinate: {self.x} meters")   
        self.y += (distance_traveled * math.sin(self.theta))      
        #self.y += (distance_traveled * math.sin(delta_theta)) 
        #For smooth curved motion
        #self.y = self.y + (distance_traveled * math.sin(self.theta + (delta_theta / 2))) 
        print(f"Y-Coordinate: {self.y} meters")
        self.theta += delta_theta
        self.theta = self.theta % (2 * math.pi)
        print(f"Theta: {math.degrees(self.theta)} degrees")

def extract_count(data):
    match = re.search(r'C=(-?\d+)', data)
    if match:
        extracted_number = int(match.group(1))
        return extracted_number
    else:
        print("No number found")
        return None

def read_encoder_data():
    ser.write(b'?C 1\r')
    #Right Wheel (Forward -> + vs. Backwards -> -)
    right_wheel_encoder_count = ser.readline().decode()
    right_count = extract_count(right_wheel_encoder_count)

    ser.write(b'?C 2\r')
    #Left Wheel (Forward -> - vs. Backwards -> +)
    left_wheel_encoder_count = ser.readline().decode()
    left_count = extract_count(left_wheel_encoder_count)
    if left_count <= 0:
        left_count = abs(left_count)
    else:
        left_count = -left_count
        
    return left_count, right_count

if __name__ == "__main__":
    test_odom = Odom()
    # test_odom.update_odom(200, 200)
    # test_odom.update_odom(100, 300) #Simulated rotation in robot's own footprint
    # test_odom.update_odom(200, 200)
    # test_odom.update_odom(0, 0)
    # test_odom.update_odom(-200, -200)

    ser.write(b'!C 1 0\r')
    ser.readline()
    ser.write(b'!C 2 0\r')
    ser.readline()

    while True:
        # Read encoder counts
        left_ticks, right_ticks = read_encoder_data()
        #print(f"Left Enocoder Counts: {left_ticks}, Right Encoder Counts: {right_ticks}")

        test_odom.update_odom(left_ticks, right_ticks)