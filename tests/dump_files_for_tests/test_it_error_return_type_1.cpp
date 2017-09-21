#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"

int main(int argc, char **argv)
{
  sensor_msgs::Range r;
  geometry_msgs::TwistStamped ts;

  r.range = ts.twist.linear.x;
}
