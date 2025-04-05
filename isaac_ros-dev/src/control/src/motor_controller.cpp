#include "motor_controller.hpp"

#define SERIAL_PORT "/dev/ttyACM0"

// constructor
MotorController::MotorController(){
  char errorOpening = motorSerial.openDevice(SERIAL_PORT, 115200);
  //motorSerial.write("!MG\r");
  if (errorOpening!=1){
    printf ("Unsuccessful connection to %s\n",SERIAL_PORT);
  }
  else{
    printf ("Successful connection to %s\n",SERIAL_PORT);
  }
  printf ("Return Code: %c\n", errorOpening);

}

// moves the robot forward
void MotorController::forward(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = -1 * (int)(stepSize * speed);
    int rightMotorSpeed = (int)(stepSize * speed);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot in backwards
void MotorController::backward(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = (int)(stepSize * speed);
    int rightMotorSpeed = -1 * (int)(stepSize * speed);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot left
void MotorController::turnLeft(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = (int)(stepSize * left_turn_speeds.first);
    int rightMotorSpeed = (int)(stepSize * left_turn_speeds.second);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot right
void MotorController::turnRight(){
  if (speed < 0){
    return;
  }
  else{
    int leftMotorSpeed = (int)(stepSize * right_turn_speeds.first);
    int rightMotorSpeed = (int)(stepSize * right_turn_speeds.second);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
  }
}

// moves the robot at specific motor speeds
void MotorController::move(float right_speed, float left_speed){
  
    int leftMotorSpeed = (int)(stepSize * left_speed);
    int rightMotorSpeed = (int)(stepSize * right_speed);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.writeString(leftMotorCommand.c_str());
    motorSerial.writeString(rightMotorCommand.c_str());
}

// stops the robot from moving
void MotorController::stop(){
  std::string leftMotorCommand = "!G 1 0\r";
  std::string rightMotorCommand = "!G 2 0 \r";
  motorSerial.writeString(leftMotorCommand.c_str());
  motorSerial.writeString(rightMotorCommand.c_str());
}

// ends the serial connection
void MotorController::shutdown(){
  std::string command = "!EX\r";
  motorSerial.writeString(command.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(1));
  motorSerial.closeDevice();
}

// updates the step size
void MotorController::setStepSize(int size){
  stepSize = size;
}

// gets the step size
int MotorController::getStepSize(){
  return stepSize;
}

// updates the speed
void MotorController::setSpeed(int s){
  speed = s;
}

// gets the speed
int MotorController::getSpeed(){
  return speed;
}

int MotorController::getLeftMotorRPM(){
  std::string command = "?C 1\r";
  char readBuffer[15] = {};
  motorSerial.writeString(command.c_str());  
  motorSerial.readString(readBuffer, '\n', 20, 1000);

  std::string encoderCount = "";
    bool equalSign = false;
    for (int i = 0; i < 15; i++) {
        std::cout << readBuffer[i] << std::endl;

        if (readBuffer[i] >= 48 && readBuffer[i] <= 57 &&  equalSign) {
          encoderCount += readBuffer[i];
        }
        if (readBuffer[i] == 61) {
            equalSign = true;
        }
    }

  return std::stoi(encoderCount);
}

int MotorController::getRightMotorRPM(){
  std::string command = "?C 2\r";
  char readBuffer[15] = {};
  motorSerial.writeString(command.c_str());  
  motorSerial.readString(readBuffer, '\n', 20, 1000);

  std::string encoderCount = "";
    bool equalSign = false;
    for (int i = 0; i < 15; i++) {
        std::cout << readBuffer[i] << std::endl;

        if (readBuffer[i] >= 48 && readBuffer[i] <= 57 &&  equalSign) {
          encoderCount += readBuffer[i];
        }
        if (readBuffer[i] == 61) {
            equalSign = true;
        }
    }

  return std::stoi(encoderCount);
}
