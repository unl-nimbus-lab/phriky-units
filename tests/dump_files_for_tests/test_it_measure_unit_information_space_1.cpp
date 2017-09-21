#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  float pi;
  pi = 3.14;
  a_sensor_msgs_Range.range = pi + 3.3;
  return  a_sensor_msgs_Range.range;
}
