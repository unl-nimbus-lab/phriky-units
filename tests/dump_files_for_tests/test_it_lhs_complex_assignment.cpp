#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{


a_sensor_msgs_Range.range = 0;


// SCOPE

float r = a_geometry_msgs_TwistStamped.twist.linear.x;
a_geometry_msgs_TwistStamped.twist.linear.y = r;

  while(true)
  {
    float r = a_geometry_msgs_TwistStamped.twist.linear.x;
    a_sensor_msgs_Range.range = r;
  }

  return 0;
}
