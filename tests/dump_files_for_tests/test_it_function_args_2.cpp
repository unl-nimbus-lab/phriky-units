#include <sensor_msgs/Range.h>
#include <ros/ros.h>

float f1(float arg1, float arg2, float arg3)
{
  return arg1 + arg2 + arg3;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range r;

  float r1= f1(r.range, r.range, r.range);  // SHOULD GET METERS

}
