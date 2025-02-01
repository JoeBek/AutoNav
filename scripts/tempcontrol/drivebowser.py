
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
    # motors = MotorController(port)
    mode = Mode.REPL


    # main loop
    # Start the listener
    
    loop = asyncio.get_running_loop()
    listener = keyboard.Listener(on_press=lambda key: loop.call_soon_threadsafe(remote.on_press, key))
    listener.start()
    # Run the processing concurrently
    await loop_co(remote, mode)


async def loop_co(remote:Remote, mode:Mode):

    
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
        --------------------------"""
            
            print(control_info)
            
            remote.listening = True
            if not remote.inputs.emtpy():
                command = await remote.inputs.get()
            
            # switch command TODO
            #TODO TODO 

            if command == Command.FORWARD:
                #motors.forward()
                print(": forward")
            elif command == Command.LEFT:
                #motors.turnLeft()
                print(": left")
            elif command == Command.RIGHT:
                print(": right")
            elif command == Command.BACKWARD:
                print(": backward")
            elif command == Command.SPEED_DOWN:
                print(": speed down")
            elif command == Command.SPEED_UP:
                print(": speed up")
            elif command == Command.NONE:
                print(": none")
            elif command == Command.STOP:
                print(": stop")
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
                sys.exit("shuttin er down")

    

async def repl():
    '''
    param: motors:Motorcontroller
    '''
    
    options = """
    options: 
    [q] exit the program
    [0] return to control mode
    [1] set left turn speeds
    [2] set right turn speeds
    
    """
    while(True):

        response = await aioconsole.ainput(options)

        if response == 'q':
            return 1
        elif response == '0':
            return 0
        elif response == '1':
            left_str = await aioconsole.ainput("input left wheel speed (-100,100): \n")
            right_str = await aioconsole.ainput("input right wheel speed (-100,100): \n")
            print(f"new wheel speeds for left turn: ({left_str},{right_str})")




if __name__ == "__main__":
    asyncio.run(main())