"""
===================================================================================================================
                                                INSTRUCTIONS
===================================================================================================================

To utilize this API, put the line "from MotorControlAPI import MotorController" at the top of your file.

To instantiate a motor controller object, use the command:
    myVar = MotorController('COMx')
where 'COMx' is replaced with the COM port of the motor controller.

This class has the following functions for moving the robot:
    myVar.forward(speed)
    myVar.backward(speed)
    myVar.turnLeft(speed)
    myVar.turnRight(speed)
for these functions, the input variable "speed" is an integer with a range of 0-100 where 100 is full speed and 0 is stopped.
Additionally, "forward" is defined as the direction of the small wheels on the robot.

The class has 2 functions for stopping the robot:
    myVar.stop()
    myVar.shutDown()

The stop() function is the primary way to get the robot to stop moving. The shutDown() command closes the
    serial port for the motor controller and should only be used as an emergency stop command or as the
    last command of your script.

===================================================================================================================
"""
import serial
import time

class MotorController:
    def __init__(self, comPort, stepsize=10):
        # Setting up serial communication with the motor controller
        self.MotorSerialObj = serial.Serial(comPort) 
        self.MotorSerialObj.baudrate = 115200  # set Baud rate to 115200
        self.MotorSerialObj.bytesize = 8   # Number of data bits = 8
        self.MotorSerialObj.parity  ='N'   # No parity
        self.MotorSerialObj.stopbits = 1   # Number of Stop bits = 1
        self.MotorSerialObj.write(b'!MG\r')
        self.stepsize = stepsize 
        #self.right_turn_speeds = (-30,0)
        #self.left_turn_speeds = (0,30)
        self.right_turn_speeds = (-10,-10)
        self.left_turn_speeds = (10,10)
        self.speed = 10

    #Tells both motors to go forward (the direction of the small wheels)
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def forward(self):
        if (self.speed < 0
            ):
            exit()

        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = -1 * int(self.stepsize * self.speed) # Left motor
        motorSpeedValue2 = int(self.stepsize * self.speed)  # Right motor
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Tells both motors to go backwards (the direction of the big wheels)
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def backward(self):
        if (self.speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(self.stepsize*self.speed)
        motorSpeedValue2 = -1 * int(self.stepsize*self.speed)
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Turns the robot to the left, resulting in the right wheel turning and the left wheel stationary
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def turnLeft(self):
        if (self.speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        #motorSpeedValue1 = int(5*speed)
        left,right = self.left_turn_speeds
        motorSpeedValue1 = left * self.stepsize
        motorSpeedValue2 = right * self.stepsize
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Turns the robot to the right, resulting in the left wheel turning and the right wheel stationary
    # speed takes values between 0-100 (0 being stopped, 100 being full speed)
    def turnRight(self):
        if (self.speed < 0):
            exit()
        
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        left, right = self.right_turn_speeds
        motorSpeedValue1 = self.stepsize * left
        motorSpeedValue2 = self.stepsize * right
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Sets the speed of the robot's wheels to a custom value specified by right and left speed
    # the speed values represent a percentage of maximum speed and can be negative
    # IMPORTANT NOTE: turning the left and right wheel in opposite directions causes the robot to be stuck in place
    def move(self, right_speed, left_speed):
        motorCommandString1 = "!G 1 "
        motorCommandString2 = "!G 2 "
        motorSpeedValue1 = int(self.stepsize*left_speed) # Left motor
        motorSpeedValue2 = int(self.stepsize*right_speed)  # Right motor
        motorCommandString1 += str(motorSpeedValue1) + "\r"
        motorCommandString2 += str(motorSpeedValue2) + "\r"
        self.MotorSerialObj.write(bytes(motorCommandString1, 'utf-8'))
        self.MotorSerialObj.write(bytes(motorCommandString2, 'utf-8'))

    #Tells motors to stop
    def stop(self):
        self.MotorSerialObj.write(b'!G 1 0\r')
        self.MotorSerialObj.write(b'!G 2 0\r')

    #Emergency shuts down motors and closes the serial port
    def shutDown(self):
        self.MotorSerialObj.write(b'!EX\r')
        time.sleep(1)
        self.MotorSerialObj.close()      # Close the port

    def set_stepsize(self,stepsize):
        self.stepsize = stepsize
    
    def get_stepsize(self):
        return self.stepsize
    

