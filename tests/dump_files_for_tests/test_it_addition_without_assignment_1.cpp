#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  geometry_msgs::Twist tw;
  if ( (tw.linear.x + tw.angular.y) > 7)
    return 42;
  return 0;
}




