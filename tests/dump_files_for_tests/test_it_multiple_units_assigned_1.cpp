#include "geometry_msgs/Accel.h"

int main(int argc, char **argv)
{

geometry_msgs::Accel a_geometry_msgs_Accel;

a_geometry_msgs_Accel.linear.x = 0;

// MULT SAME UNITS
a_geometry_msgs_Accel.linear.x = a_geometry_msgs_Accel.linear.x * a_geometry_msgs_Accel.linear.x; // MULTIPLE UNITS ASSIGNED

  return 0;
}





