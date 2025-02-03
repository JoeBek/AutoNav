
from MotorControlAPI import MotorController
import sys
import serial
import time

from enum import Enum
from RemoteControl import Command
from pynput import keyboard




def main():

    # get sys args
    args = sys.argv[1:]

    if len(args) > 1:
        bluetooth_port = args[0]
        motor_port = args[1]

    else:
        sys.exit("you forgot one of the ports silly (bluetooth then motors)")


    reciever = serial.Serial(bluetooth_port) 
    reciever.baudrate = 115200  # set Baud rate to 115200
    reciever.bytesize = 8   # Number of data bits = 8
    reciever.parity  ='N'   # No parity
    reciever.stopbits = 1   # Number of Stop bits = 1


    motors = MotorController(motor_port)
    mode = Mode.REPL

    # main loop
    
    loop_co(motors, mode)


class Mode(Enum):
    REPL = 0
    CONTROL = 1



def loop_co(motors:MotorController, receiver, mode:Mode):

    
    while(True):

        # switch(mode):        ... I wish

        

        if mode == Mode.CONTROL:
            control_info = """\n
        how to play:
        --------------------------
            w    |   or arrow keys
         a  s  d |   to control the direction
        --------------------------
        e - speed up
        q - speed down
        r - go into options mode
        space - stop
        --------------------------"""
            
            
            
            print(control_info)
            
            # following line is future work for allowing single movement presses to act as toggles
            # if not remote.inputs.emtpy():

            # block until a command is read
            while receiver.inWaiting < 0:
                time.sleep(.1)
                

            command_str = receiver.readline().decode('utf-8').strip()
            
            command = Command[command_str]

            
            # switch command TODO
            #TODO TODO 

            if command == Command.FORWARD:
                #motors.forward()
                print(": forward")
                motors.forward()
            elif command == Command.LEFT:
                #motors.turnLeft()
                print(": left")
                motors.turnLeft()
            elif command == Command.RIGHT:
                print(": right")
                motors.turnRight()
            elif command == Command.BACKWARD:
                print(": backward")
                motors.backward()
            elif command == Command.SPEED_DOWN:
                motors.speed -= 1
                print(f": speed down | new speed: {motors.speed}")
                command = Command.NONE # avoid no-input speed up/down loop
            elif command == Command.SPEED_UP:
                motors.speed += 1
                print(f": speed up | new speed: {motors.speed}")
                command = Command.NONE
            elif command == Command.NONE:
                print(": none")
            elif command == Command.STOP:
                print(": stop")
                motors.stop()
            elif command == Command.CHANGE_MODE:
                print(": change mode")
                mode = Mode.REPL
            
            

        elif mode == Mode.REPL:
            print()
            code = repl(motors)
            # 0, return to control; 1, exit program
            if code == 0:
                mode = Mode.CONTROL
                receiver.flush() # flush buffer for commands written in repl mode
            elif code == 1:
                motors.shutDown()
                sys.exit("shuttin er down")

    

async def repl(motors:MotorController):
    '''
    param: motors:Motorcontroller
    '''
    
    options = """
    options: 
    [q] exit the program
    [0] return to control mode
    [1] set left turn speeds
    [2] set right turn speeds
    [3] set speed (1-100)
    
    """
    while(True):

        response = input(options)

        if response == 'q':
            return 1
        elif response == '0':
            return 0
        elif response == '1':
            left_str = input("input left wheel speed for left turn (-100,100): \n")
            right_str = input("input right wheel speed for right turn (-100,100): \n")

            # assume clean input
            
            motors.left_turn_speeds = (int(left_str), int(right_str))
            print(f"new wheel speeds for left turn: ({left_str},{right_str})")

        elif response == '2':
            left_str = input("input left wheel speed for right turn (-100,100): \n")
            right_str = input("input right wheel speed for right turn (-100,100): \n")

            # assume clean input
            
            motors.right_turn_speeds = (int(left_str), int(right_str))
            print(f"new wheel speeds for left turn: ({left_str},{right_str})")

        elif response == '3':

            speedstr = input("set new speed (1-100)")

            # if you are reading this please observe my funny and creative variable name
            motors.speed = int(speedstr)
            print(f"new motor speed set to: {motors.speed}")



    
if __name__ == "__main__":
    main()
    