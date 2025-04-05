
import datetime
from serial import Serial

test_number = 1 ## Change this to change the Test you want to run
TEST1_DURATION = 5
TEST2_DURATION = 10

baudrate = 115200
cereal = Serial('/dev/ttyACM0', baudrate, timeout=1)
cereal.stopbits = 1
cereal.parity = 'N'
cereal.bytesize = 8
stepsize = 10


def send_message(cereal, message:str):
    message += '\r'
    cereal.write(message.encode('utf-8'))
    print(f"sending: {message}")

def get_BatteryCurrent():
    motorCommandString = "?BA"
    send_message(cereal, motorCommandString)
    while cereal.in_waiting <= 0 :
        pass
    while cereal.in_waiting > 0:
        print("Reading...")
        line = cereal.readline().decode('utf-8')
    with open ('CurrentReadings.txt', 'a') as file:
        file.writelines(line)

def get_RPMs():
    motorCommandString1 = '?S 1'
    motorCommandString2 = '?S 2'
    send_message(cereal, motorCommandString1)
    while cereal.in_waiting <= 0:
        pass
    while cereal.in_waiting > 0:
        line1 = cereal.readline().decode('utf-8')
    send_message(cereal, motorCommandString2)
    while cereal.in_waiting <= 0:
        pass
    while cereal.in_waiting > 0:
        line2 = cereal.readline().decode('utf-8')
    with open ('RPMReadings.txt', 'a') as file:
        lines = (line1,line2)
        file.writelines(lines)

def forward(speed):
    if (speed < 0
        ):
        exit()

    
    motorCommandString1 = "!G 1 "
    motorCommandString2 = "!G 2 "
    motorSpeedValue1 = -1 * int(stepsize * speed) # Left motor
    motorSpeedValue2 = int(stepsize * speed)  # Right motor
    motorCommandString1 += str(motorSpeedValue1) + "\r"
    motorCommandString2 += str(motorSpeedValue2) + "\r"
    print(motorCommandString1)
    print(motorCommandString2)
    cereal.write(bytes(motorCommandString1, 'utf-8'))
    cereal.write(bytes(motorCommandString2, 'utf-8'))

## Test Number 1 (1mph)
if(test_number == 1):
    start_time = datetime.datetime.now()
    time = datetime.datetime.now()
    duration = datetime.timedelta(seconds=TEST1_DURATION)
    while (time - start_time < duration):
        print("Moving...")
        forward(20)
        time = datetime.datetime.now()
    get_BatteryCurrent()
    get_RPMs()
## Test Number 2 (5mph)
if(test_number == 2):
    start_time = datetime.datetime.now()
    time = datetime.datetime.now()
    duration = datetime.timedelta(seconds=TEST2_DURATION)
    while (time - start_time < duration):
        forward(20)
        get_BatteryCurrent()
        get_RPMs()
        time = datetime.datetime.now()


