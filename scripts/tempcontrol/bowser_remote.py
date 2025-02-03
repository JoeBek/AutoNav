
from RemoteControl import Remote
import serial
import sys


def main():
    # get sys args
    args = sys.argv[1:]

    if len(args) > 0:
        port = args[0]
    else:
        sys.exit("you forgot the port silly")


    ser = serial.Serial(port) 
    ser.baudrate = 115200  # set Baud rate to 115200
    ser.bytesize = 8   # Number of data bits = 8
    ser.parity  ='N'   # No parity
    ser.stopbits = 1   # Number of Stop bits = 1

    
    remote = Remote(ser=ser)
    remote.start_serial()


    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("good bye")
        ser.close()



if __name__ == "__main__":
    main()


