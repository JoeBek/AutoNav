import serial
import time
from serial import Serial




# serial parameters
baudrate = 115200 
cereal = Serial('/dev/ttyUSB0', baudrate, timeout=3)
cereal.stopbits = 1
cereal.parity = 'N'
cereal.bytesize = 8

encoding = 'ascii'

def send_message(cereal, message:str):
    message += '\n'
    cereal.write(message.encode(encoding))
    print(f"sending: {message}")

def send_message_gps(cereal, message:str):
    message +='\r\n'
    cereal.write(message.encode(encoding))
    print(f"sending to gps: {message}")


    
try:
    while True:

        message = input("enter message: \n")

        if message == 'break':
            break

        if message:
            send_message_gps(cereal, message)

        time.sleep(.4)
        # recieve messages
        
        while cereal.in_waiting <= 0:
            pass


        print("receiving something ... \n")
        line = cereal.readline().decode(encoding, errors='ignore')
        print(line)
        
    
except KeyboardInterrupt:
    pass
finally:
    cereal.close()

