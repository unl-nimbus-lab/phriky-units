#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char **argv)
{
  float f = 3.14;
  geometry_msgs::Quaternion q;
  float x = f + q.x;
}
