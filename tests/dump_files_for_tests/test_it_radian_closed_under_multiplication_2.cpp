#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  float f = std::atan(10); // f SHOULD GET RADIANS
  geometry_msgs::Twist tw;
  tw.linear.z = f * f;
}
