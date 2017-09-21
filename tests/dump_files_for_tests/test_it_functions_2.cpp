#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


float f1(sensor_msgs::Range &r)
{
  float return_value = r.range + 3.14;
  return return_value;
}

float f1(float r)
{
  float return_value = r*r;
  return return_value;
}


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r = a_sensor_msgs_Range.range;

  float y = f1(a_sensor_msgs_Range);
  float z = f1(r);

  return 0;
}
