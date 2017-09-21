#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
  
  geometry_msgs::Twist t;
  t.linear.x= 0;

  float f = t.linear.x / t.linear.x;
  float x = std::sqrt(f);  // UNITLESS?

  return 0;

}
