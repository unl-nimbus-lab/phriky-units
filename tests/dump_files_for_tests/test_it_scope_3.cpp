#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{

geometry_msgs::TwistStamped a_geometry_msgs_TwistStamped;
sensor_msgs::Range a_sensor_msgs_Range;

a_geometry_msgs_TwistStamped.twist.linear.x = 0;
a_sensor_msgs_Range.max_range = 0;


// SCOPE

float r = a_geometry_msgs_TwistStamped.twist.linear.x;
float s = r;
float t = s;


  while(true)
  {
    float j = s;
    float r = a_sensor_msgs_Range.max_range;
    s = r;
  }
}
