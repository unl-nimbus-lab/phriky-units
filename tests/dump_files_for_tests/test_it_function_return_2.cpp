#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


float f1(float r)
{
  sensor_msgs::Range b;
  b.header.stamp.sec = 42;
  return r / b.header.stamp.sec;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 3.14;

  // SCOPE

  float x = f1(a_sensor_msgs_Range.range);
  a_sensor_msgs_Range.range=3.24;
  float y = f1(a_sensor_msgs_Range.range);

  return 0;
}
