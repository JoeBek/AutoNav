from RemoteControl import Remote
import sys
import serial


def main():


    # SET PORT HERE FOR DEFAULT BEHAVIOR
    port = "/dev/ttyTCU0"

    if (len(sys.argv) > 0):
        port = sys.argv[0]


    cereal = serial.Serial(port) 
    cereal.baudrate = 115200  # set Baud rate to 115200
    cereal.bytesize = 8   # Number of data bits = 8
    cereal.parity  ='N'   # No parity
    cereal.stopbits = 1   # Number of Stop bits = 1
        
    remote = Remote(cereal) # I love abstraction

    remote.start_serial()
