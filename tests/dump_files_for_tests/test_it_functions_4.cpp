#include <sensor_msgs/Range.h>
#include <ros/ros.h>

int f1(float f)
{
  return 17; 
}

int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r = (float) f1(a_sensor_msgs_Range.range);  // SHOULD GET NO UNITS

}
