//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "motor_controller.hpp"

class Autonomous {
private:
  
public:
 
  double linearX;
  double linearY;
  double linearZ;
  
  double angularZ;

  Autonomous();
  void goToPose(double relativeX, double relativeY, MotorController& motors);
  
  double one_roatation = 125000;
};