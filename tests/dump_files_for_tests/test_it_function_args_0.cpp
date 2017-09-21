#include <sensor_msgs/Range.h>
#include <ros/ros.h>

float f1(float arg1)
{
  return arg1;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range r;

  float r1= f1(r.range);  // SHOULD GET METERS

}
