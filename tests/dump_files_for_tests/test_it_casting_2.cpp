#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
  
  sensor_msgs::Range r1;
  r1.range = 0;
  int j = 1;
  
  float f = (float)(r1.range + (float) j);


}
