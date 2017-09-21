#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{
  sensor_msgs::Range r;

  int x = 1;
  int y = 2;

  if ( x < y && 20 < r.range )
  {
    x++;
  }

}
