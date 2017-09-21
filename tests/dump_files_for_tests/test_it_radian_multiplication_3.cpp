#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
  float f = std::atan(10); // f SHOULD GET RADIANS
  geometry_msgs::Twist tw;
  tw.linear.x = tw.linear.y * f;
}
