#include "motor_controller.hpp"

// constructor
MotorController::MotorController(){

  if (!motorSerial.isOpen()) {
      std::cerr << "Failed to connect to motor controller on " << comPort << std::endl;
  }
  else{
    motorSerial.write("!MG\r");
  }
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

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
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

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
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

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
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

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
  }
}

// moves the robot at specific motor speeds
void MotorController::move(float right_speed, float left_speed){
  
    int leftMotorSpeed = (int)(stepSize * left_speed);
    int rightMotorSpeed = (int)(stepSize * right_speed);

    std::string leftMotorCommand = "!G 1" + std::to_string(leftMotorSpeed) + "\r";
    std::string rightMotorCommand = "!G 2" + std::to_string(rightMotorSpeed) + "\r";

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
}

// stops the robot from moving
void MotorController::stop(){
  motorSerial.write("!G 1 0\r");
  motorSerial.write("!G 2 0 \r");
}

// ends the serial connection
void MotorController::shutdown(){
  motorSerial.write("!EX\r");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  motorSerial.close();
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
  motorSerial.write("?S 1\r");
  std::string buffer;
  motorSerial.read(buffer);
  return std::stoi(motorSerial.last_string);
}

int MotorController::getRightMotorRPM(){
  motorSerial.write("?S 2\r");
  std::string buffer;
  motorSerial.read(buffer);
  return std::stoi(motorSerial.last_string);
}