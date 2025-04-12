#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Autonomous {
private:
  
public:
 
  double positionX;
  double positionY;
  double positionZ;
  
  double orientationX;
  double orientationY;
  double orientationZ;
  double orientationW;

  double getYawFromQuaternion(double x, double y, double z, double w);

};