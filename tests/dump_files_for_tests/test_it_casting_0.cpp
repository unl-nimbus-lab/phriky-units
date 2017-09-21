#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
  
  sensor_msgs::Range r;
  r.range = 0;
  
  float f = (float) r.range;


}
