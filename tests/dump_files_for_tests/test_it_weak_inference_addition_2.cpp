#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 orTmp(quat);
  orTmp.getRPY(roll, pitch, yaw);
}

int main(int argc, char **argv)
{
  double r, p, y;
  double unknown_var;
  double x, z;
  unknown_var = 0;
  x=0;
  z=0;
  geometry_msgs::Twist tw;
  tf2::Quaternion myQuat;
  quatToRPY(myQuat, r, p, y);
  tw.linear.x = unknown_var + r;
  tw.linear.y = unknown_var * x * z + r;
  return 0;
}




