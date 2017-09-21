#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  geometry_msgs::Twist tw;
  if ( 7 > (tw.linear.x - tw.angular.y))
    return 42;
  return 0;
}




