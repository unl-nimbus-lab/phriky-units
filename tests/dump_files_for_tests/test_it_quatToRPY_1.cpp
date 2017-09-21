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
  geometry_msgs::Twist tw;
  tf2::Quaternion myQuat;
  quatToRPY(myQuat, r, p, y);
  tw.linear.x = r;
  return 0;
}




