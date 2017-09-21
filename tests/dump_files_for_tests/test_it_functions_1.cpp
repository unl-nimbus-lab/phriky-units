#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


float f1(sensor_msgs::Range &r)
{
  float return_value = r.range + 3.14;
  return return_value;
}


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r = a_sensor_msgs_Range.range;

  float z = f1(a_sensor_msgs_Range);

  return 0;
}
