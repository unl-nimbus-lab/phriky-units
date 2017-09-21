#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r = a_sensor_msgs_Range.range;

  if ( 7 && r )
  {
    r = r + 3.14;
  }

  if ( r || 7 )
  {
    r = r + 3.14;
  }

  if ( !r )
  {
    r = r + 3.14;
  }

  return 0;
}
