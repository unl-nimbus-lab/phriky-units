#include <ros/ros.h>
#include <math.h>

int main(int argc, char **argv)
{
  
  ros::Duration d1(0.1);
  float s = std::abs(d1.toSec());
  float t = std::fabs(d1.toSec());

}
