#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


float f1(float r, float t)
{
  return r / t;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  geometry_msgs::TwistStamped twist;

  twist.header.stamp.sec = 42.0;
  a_sensor_msgs_Range.range = 3.14;

  float x = f1(a_sensor_msgs_Range.range, twist.header.stamp.sec);

  return 0;
}
