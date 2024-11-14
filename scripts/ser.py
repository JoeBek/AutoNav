import serial
import time
from serial import Serial




# serial parameters
baudrate = 9600
cereal = Serial('/dev/ttyACM0', baudrate, timeout=1)
cereal.stopbits = 1
cereal.parity = 'N'
cereal.bytesize = 8

encoding = 'ascii'

def send_message(cereal, message:str):
    message += '\n'
    cereal.write(message.encode(encoding))
    print(f"sending: {message}")

    
try:
    while True:

        message = input("enter message: \n")

        if message == 'break':
            break

        if message:
            send_message(cereal, message)

        time.sleep(.4)
        # recieve messages
        
        while cereal.in_waiting <= 0:
            pass


        print("receiving something ... \n")
        line = cereal.readline().decode(encoding)
        print(line)
        
    
except KeyboardInterrupt:
    pass
finally:
    cereal.close()

