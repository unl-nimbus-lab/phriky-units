#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <math.h>

int main(int argc, char **argv)
{
  
  sensor_msgs::Range r;
  r.range = 0;

  float f = r.range * r.range;
  float x = std::sqrt(f);  // meters?

  return 0;

}
