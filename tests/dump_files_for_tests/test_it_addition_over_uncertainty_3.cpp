#include "geometry_msgs/Accel.h"

int main(int argc, char **argv)
{

  geometry_msgs::Accel a_geometry_msgs_Accel;
  a_geometry_msgs_Accel.linear.x = 0;

  // ADDITION SAME UNITS
  float z = (42.0 * a_geometry_msgs_Accel.linear.x) + (42 * a_geometry_msgs_Accel.linear.x); // BOTH UNCERTAIN

  return 0;
}
