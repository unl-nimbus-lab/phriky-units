#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

float getX()
{
  return 0.;
}

int main(int argc, char **argv)
{
  geometry_msgs::Twist tw;
  tw.linear.x = getX();
  return 0;
}




