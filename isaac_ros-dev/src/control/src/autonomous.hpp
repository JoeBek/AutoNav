#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Autonomous {
private:

  double positionX;
  double positionY;
  double positionZ;
  double orientationX;
  double orientationY;
  double orientationZ;
  double orientationW;
  
public:
  double getCurrPositionX();
  double getCurrPositionY();
  double getCurrPositionZ();

  double getCurrOrientationX();
  double getCurrOrientationY();
  double getCurrOrientationZ();
  double getCurrOrientationW();
    
  double getYawFromQuaternion(double x, double y, double z, double w);
};