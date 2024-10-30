import serial
import time
from serial import Serial




# serial parameters
cereal = Serial('/dev/ttyACM0', 115200, timeout=1)
cereal.stopbits = 1
cereal.parity = 'N'
cereal.bytesize = 8

encoding = 'utf-8'

def send_message(cereal, message:str):
    cereal.write(message.encode(encoding) + b'\r')
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
        
        if cereal.in_waiting > 0:
            print("receiving something ... \n")
            line = cereal.readline().decode(encoding)
            print(line)
    
except KeyboardInterrupt:
    pass
finally:
    cereal.close()

