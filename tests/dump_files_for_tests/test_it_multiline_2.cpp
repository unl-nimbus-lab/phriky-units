#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::Time t;
  float dt = 0.;
  geometry_msgs::Twist robot;
  float f;
  f = 0;

  f = robot.linear.x * f;  // INSUFFICIENT  ALL ON ONE LINE

  f = 
     robot.linear.x + f; // INSUFFICIENT  ADDITION (DOESN'T MODIFY THE STATE OF 'f' EACH TIME)

  f = 
     robot.linear.x * dt;  // SUFFICIENT   MULTIPLICATION

  f = 
     dt / robot.linear.x;  // SUFFICIENT

  f = 
     dt / t.toSec();  // SUFFICIENT

  
  return 0;
}





