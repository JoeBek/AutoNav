#include "motor_controller.hpp"

// contstructor
MotorController::MotorController(std::string comPort, int stepsize = 10){
  motorSerial = new motorSerial(comPort, 115200);
  stepSize = stepSize; 

  if (!motorSerial.isConnected()) {
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

    std::string leftMotorCommand = "!G 1" + leftMotorSpeed + "\r";
    std::string rightMotorCommand = "!G 2" + rightMotorSpeed + "\r";

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

    std::string leftMotorCommand = "!G 1" + leftMotorSpeed + "\r";
    std::string rightMotorCommand = "!G 2" + rightMotorSpeed + "\r";

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

    std::string leftMotorCommand = "!G 1" + leftMotorSpeed + "\r";
    std::string rightMotorCommand = "!G 2" + rightMotorSpeed + "\r";

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

    std::string leftMotorCommand = "!G 1" + leftMotorSpeed + "\r";
    std::string rightMotorCommand = "!G 2" + rightMotorSpeed + "\r";

    motorSerial.write(leftMotorCommand);
    motorSerial.write(rightMotorCommand);
  }
}

// moves the robot at specific motor speeds
void MotorController::move(float right_speed, float left_speed){
  
    int leftMotorSpeed = (int)(stepSize * left_speed);
    int rightMotorSpeed = (int)(stepSize * right_speed);

    std::string leftMotorCommand = "!G 1" + leftMotorSpeed + "\r";
    std::string rightMotorCommand = "!G 2" + rightMotorSpeed + "\r";

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
void MotorController::getStepSize(){
  return stepSize;
}

// updates the speed
void MotorController::setSpeed(int s){
  speed = s;
}

// gets the speed
void MotorController::getSpeed(){
  return speed;
}
