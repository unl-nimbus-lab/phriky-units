#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  geometry_msgs::Twist tw;
  float x;
  x=0;
  if ((tw.linear.x + (tw.angular.y * x)) > 7)
    return 42;
  return 0;
}




