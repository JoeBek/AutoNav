
import sys
import asyncio
from enum import Enum
from MotorControlAPI import MotorController
from RemoteControl import Remote, Command
from pynput import keyboard
import aioconsole

class Mode(Enum):
    REPL = 0
    CONTROL = 1


async def main():

    # get sys args
    args = sys.argv[1:]

    if len(args) > 0:
        port = args[0]

    # make necessary objects

    command = Command.STOP # only ever one command
    remote = Remote()
    motors = MotorController(port)
    mode = Mode.REPL


    # main loop
    # Start the listener
    
    loop = asyncio.get_running_loop()
    listener = keyboard.Listener(on_press=lambda key: loop.call_soon_threadsafe(remote.on_press, key))
    listener.start()
    # Run the processing concurrently
    await loop_co(motors, remote, mode)


async def loop_co(motors:MotorController, remote:Remote, mode:Mode):

    
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
            
            remote.listening = True
            # following line is future work for allowing single movement presses to act as toggles
            # if not remote.inputs.emtpy():
            command = await remote.inputs.get()
            
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
            remote.listening = False
            print()
            code = await repl()
            # 0, return to control; 1, exit program
            if code == 0:
                mode = Mode.CONTROL
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