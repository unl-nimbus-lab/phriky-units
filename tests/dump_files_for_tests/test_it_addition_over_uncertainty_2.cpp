#include "geometry_msgs/Accel.h"

int main(int argc, char **argv)
{

  geometry_msgs::Accel a_geometry_msgs_Accel;
  a_geometry_msgs_Accel.linear.x = 0;

  // ADDITION SAME UNITS
  float y = (42.0 * a_geometry_msgs_Accel.linear.x) + a_geometry_msgs_Accel.linear.x;  // ONE CERTAIN

  return 0;
}
