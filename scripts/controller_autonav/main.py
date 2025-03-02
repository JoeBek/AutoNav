
import sys
import asyncio
import pygame
from enum import Enum
from MotorControlAPI import MotorController
from RemoteControl import Remote, Command
import aioconsole

class Mode(Enum):
    REPL = 0
    CONTROL = 1


async def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    #print(f"Connected to controller: {joystick.get_name()}")

    args = sys.argv[1:]
    if len(args) > 0:
        port = args[0]

    command = Command.STOP
    remote = Remote()
    motors = MotorController(port)
    mode = Mode.CONTROL  # Start in CONTROL mode

    await loop_co(motors, remote, mode, joystick)


async def loop_co(motors: MotorController, remote: Remote, mode: Mode, joystick):
    while True:
        #print(f"Current mode: {mode}")  # Debugging output
        pygame.event.pump()

        if mode == Mode.CONTROL:
            command, leftSpeed, rightSpeed = get_joystick_command(joystick)

            if command != Command.NONE:
                #print(f"Command received: {command}")
                pass

            if command == Command.MOVE:
                #print("Sending command: MOVING")
                motors.move(rightSpeed * motors.speed, leftSpeed * motors.speed)
            elif command == Command.SPEED_DOWN:
                motors.speed -= 10
                #print(f"Speed Down | New speed: {motors.speed}")
            elif command == Command.SPEED_UP:
                motors.speed += 10
                #print(f"Speed Up | New speed: {motors.speed}")
            elif command == Command.STOP:
                #print("Sending command: STOP")
                motors.shutDown()
            elif command == Command.CHANGE_MODE:
                mode = Mode.REPL  # Switch mode
            
        elif mode == Mode.REPL:
            print("Entering REPL mode")  # Debugging output
            remote.listening = False
            code = await repl(motors)

            if code == 0:
                mode = Mode.CONTROL  # Return to control mode
            elif code == 1:
                motors.shutDown()
                sys.exit("shutting down")


    
def get_joystick_command(joystick):
    """Reads Xbox controller inputs and returns the corresponding command."""
    pygame.event.pump()  # Process joystick events

    # Joystick axis values (-1 to 1)
    left_y = joystick.get_axis(1)  # Left stick vertical
    right_y = joystick.get_axis(3)  # Left stick horizontal
    btn_a = joystick.get_button(0)  # A - Stop
    btn_rb = joystick.get_button(5)  # Right bumper - Speed up
    btn_lb = joystick.get_button(4)  # Left bumper - Speed down
    btn_x = joystick.get_button(2) # X - button`            `

    # Print debug info
    #print(f"Left Joystick: {left_y}, Right Joystick: {right_y}, A: {btn_a}, RB: {btn_rb}, LB: {btn_lb}")

    # To account for the middle positions not being exactly 0
    if left_y < 0.1 and left_y > -0.1:
        left_y = 0
    
    if right_y < 0.1 and right_y > -0.1:
        right_y = 0
    
    if right_y != 0 or left_y != 0:
        return Command.MOVE, left_y, right_y

    # Speed control
    if btn_rb:
        return Command.SPEED_UP, left_y, right_y
    if btn_lb:
        return Command.SPEED_DOWN, left_y, right_y

    # Stop command
    if btn_a:
        return Command.STOP, left_y, right_y
    
    if btn_x:
        return Command.GET_CURRENT

    return Command.NONE, left_y, right_y


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

        response = await aioconsole.ainput(options)

        if response == 'q':
            return 1
        elif response == '0':
            return 0
        elif response == '1':
            left_str = await aioconsole.ainput("input left wheel speed for left turn (-100,100): \n")
            right_str = await aioconsole.ainput("input right wheel speed for right turn (-100,100): \n")

            # assume clean input
            
            motors.left_turn_speeds = (int(left_str), int(right_str))
            print(f"new wheel speeds for left turn: ({left_str},{right_str})")
        elif response == '2':
            left_str = await aioconsole.ainput("input left wheel speed for right turn (-100,100): \n")
            right_str = await aioconsole.ainput("input right wheel speed for right turn (-100,100): \n")

            # assume clean input
            
            motors.right_turn_speeds = (int(left_str), int(right_str))
            print(f"new wheel speeds for left turn: ({left_str},{right_str})")

        elif response == '3':

            speedstr = await aioconsole.ainput("set new speed (1-100)")

            # if you are reading this please observe my funny and creative variable name
            motors.speed = int(speedstr)
            print(f"new motor speed set to: {motors.speed}")




if __name__ == "__main__":
    asyncio.run(main())