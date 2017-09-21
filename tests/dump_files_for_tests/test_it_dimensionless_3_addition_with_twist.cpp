#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  geometry_msgs::Twist t;
  float f = 3.14;
  float x = f + t.linear.x;
}
