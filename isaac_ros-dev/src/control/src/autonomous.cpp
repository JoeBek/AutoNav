#include "autonomous.hpp"

Autonomous::Autonomous(){
  
}

void Autonomous::goToPose(double relativeX, double relativeY, MotorController& motors){
  const double PI = 3.14159265358979323846;
  //double relativeYaw = 270;
  double initialTurnAngle = std::atan2(relativeY, relativeX) * (180 / PI);
  double totalDistance = std::sqrt((relativeX * relativeX) + (relativeY * relativeY));


  if(initialTurnAngle < -10 && initialTurnAngle > 10){
    if(initialTurnAngle < 0){
        initialTurnAngle *= -1;
        if(initialTurnAngle < 40){
            while(static_cast<double>(motors.getRightEncoderCount()) > (static_cast<double>(initialTurnAngle / 360)  * -156000)){
                motors.move(-10, 10);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        else{
            while(static_cast<double>(motors.getRightEncoderCount()) > (static_cast<double>(initialTurnAngle / 360)  * -125000)){
                motors.move(-10, 10);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
    }
    else{
        //initialTurnAngle *= -1;
        if(initialTurnAngle < 40){
            while(static_cast<double>(motors.getLeftEncoderCount()) < (static_cast<double>(initialTurnAngle / 360) *156000)){
                motors.move(10, -10);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        else{
            while(static_cast<double>(motors.getLeftEncoderCount()) < 125000){
                motors.move(10, -10);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
    }
  }

  motors.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  int totalEncoderCount = motors.getRightEncoderCount();
  //1.275 is 50.2 inches in meters
  if(totalDistance > 1.275){
      while(static_cast<double>(motors.getRightEncoderCount()) < ((static_cast<double>((totalDistance / 1.275) * 76000)) + totalEncoderCount)){
          motors.move(-10, -10);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
  }
  else {
      while(static_cast<double>(motors.getRightEncoderCount()) < ((static_cast<double>((totalDistance / 1.275) * 67800)) + totalEncoderCount)){
          motors.move(-10, -10);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
  }
  motors.stop();
}
