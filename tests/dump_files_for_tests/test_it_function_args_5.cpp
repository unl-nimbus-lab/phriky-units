#include <sensor_msgs/Range.h>
#include <ros/ros.h>

float f1(float arg1, float arg2)
{
  return arg1;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range r;
  ros::Duration d;
 

  float r1= f1(r.range, r.range);  // SHOULD GET METERS
  float r2= f1(d.toSec(), r.range);  // SHOULD GET METERS

}
