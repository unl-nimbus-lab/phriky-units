#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  
  double dt = 0.;
  geometry_msgs::Twist tw;
  tw.angular.x = 1/dt;

}
