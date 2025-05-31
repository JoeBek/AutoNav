# Python code transmits a byte to Arduino /Microcontroller
import time
from pynput import keyboard
from pynput.keyboard import Key
from enum import Enum
import serial

class Command(Enum):

    STOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    BACKWARD = 4
    SPEED_UP = 5
    SPEED_DOWN = 6
    NONE = 7
    CHANGE_MODE = 8





class Remote():

    def __init__(self, ser=None):
        self.listening = False
        self.ser = ser

    def on_key_release(self,key):
        command = Command.NONE
        if key == Key.right or key.char == "d":
            command = Command.RIGHT
        elif key == Key.left or key.char == "a":
            command = Command.LEFT
        elif key == Key.up or key.char == "w":
            command = Command.FORWARD
            print("forward")
        elif key == Key.down or key.char == "s":
            command = Command.BACKWARD
        elif key.char == "q":
            command = Command.SPEED_DOWN
        elif key.char == "e":
            command = Command.SPEED_UP
        elif key == Key.space:
            command = Command.STOP
        elif key.char == 'r':
            command = Command.CHANGE_MODE

        


    def on_press(self,key):
        if not self.listening:
            return
        command = Command.NONE
        if key == Key.right or key.char == "d":
            command = Command.RIGHT
        elif key == Key.left or key.char == "a":
            command = Command.LEFT
        elif key == Key.up or key.char == "w":
            command = Command.FORWARD
        elif key == Key.down or key.char == "s":
            command = Command.BACKWARD
        elif key.char == "q":
            command = Command.SPEED_DOWN
        elif key.char == "e":
            command = Command.SPEED_UP
        elif key == Key.space:
            command = Command.STOP
        elif key.char == 'r':
            command = Command.CHANGE_MODE

        

    def on_press_serial(self,key):
        command = Command.NONE
        if key == Key.right or key.char == "d":
            command = Command.RIGHT
        elif key == Key.left or key.char == "a":
            command = Command.LEFT
        elif key == Key.up or key.char == "w":
            command = Command.FORWARD
            print("forward")
        elif key == Key.down or key.char == "s":
            command = Command.BACKWARD
        elif key.char == "q":
            command = Command.SPEED_DOWN
        elif key.char == "e":
            command = Command.SPEED_UP
        elif key == Key.space:
            command = Command.STOP
        elif key.char == 'r':
            command = Command.CHANGE_MODE

        self.ser.write(command.name.encode('utf-8'))
        print(f"sending: {command.name}")
 
    def start_serial(self):
        listener = keyboard.Listener(
            on_press=self.on_press_serial
        )
        listener.start()

    # placeholder, dont use with async
    def is_listening(self):
        return self.listening

    

#listener = keyboard.Listener(
#    on_press=on_key_press,
#    on_release=on_key_release)
#listener.start()
