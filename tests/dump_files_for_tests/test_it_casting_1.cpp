#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
  
  sensor_msgs::Range r1;
  sensor_msgs::Range r2;
  r1.range = 0;
  r2.range = 0;
  
  float f = (float) r1.range + (float) r2.range;


}
