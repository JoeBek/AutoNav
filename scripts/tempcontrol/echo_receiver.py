'''
this file simply echoes commands from one serial port to another 
'''
import serial
import sys
import time


TIMEOUT=0.05

def main():
    

    # get sys args
    args = sys.argv[1:]

    if len(args) > 1:
        bluetooth_port = args[0]
        motor_port = args[1]

    else:
        sys.exit("you forgot one of the ports silly (bluetooth then motors)")


    receiver = serial.Serial(bluetooth_port) 
    receiver.baudrate = 115200  # set Baud rate to 115200
    receiver.bytesize = 8   # Number of data bits = 8
    receiver.parity  ='N'   # No parity
    receiver.stopbits = 1   # Number of Stop bits = 1

    sender = serial.Serial(motor_port) 
    sender.baudrate = 115200  # set Baud rate to 115200
    sender.bytesize = 8   # Number of data bits = 8
    sender.parity  ='N'   # No parity
    sender.stopbits = 1   # Number of Stop bits = 1


    while True:

        while not receiver.in_waiting:
            time.sleep(TIMEOUT)
        
        line = receiver.readline()
        print(f"received: {line.decode("utf-8")}")
        sender.write(line)




if __name__ == "__main__":
    main()
    

    