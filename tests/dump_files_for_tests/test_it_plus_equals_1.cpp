#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

int main(int argc, char **argv)
{
  double x, y, z, w;
  geometry_msgs::Pose p;

  x = p.position.x * p.position.x;
  y = p.position.x;
  z = p.position.x;
  w = p.position.x;

  x += y * (z - w);
}




