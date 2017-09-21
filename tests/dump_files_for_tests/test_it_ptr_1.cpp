#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
  geometry_msgs::Twist tw;
  
  float f = 1 > 0 ?  tw.linear.x : tw.angular.x;

}
