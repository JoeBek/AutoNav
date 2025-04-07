
double getPositionX(){
  return positionX;
}

double getPositionY(){
  return positionY;
}

double getPositionZ(){
  return positionZ;
}


double getOrientationX(){
  return orientationX;
}

double getOrientationY(){
  return orientationY;
}

double getOrientationZ(){
  return orientationZ;
}

double getOrientationW(){
  return orientationW;
}


double getYawFromQuaternion(double x, double y, double z, double w) {
  tf2::Quaternion quat(x, y, z, w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}
