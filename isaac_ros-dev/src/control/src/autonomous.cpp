#include "autonomous.hpp"


double getYawFromQuaternion() {
  tf2::Quaternion quat(orientationX, orientationY, orientationZ, orientationW);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}
