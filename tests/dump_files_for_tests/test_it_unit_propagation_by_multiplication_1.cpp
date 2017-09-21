#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"

int main(int argc, char **argv)
{

geometry_msgs::Accel a_geometry_msgs_Accel;
geometry_msgs::AccelStamped a_geometry_msgs_AccelStamped;

a_geometry_msgs_AccelStamped.accel.linear.x = 0;
a_geometry_msgs_Accel.linear.x = 0;

// MULT SAME UNITS
float f = a_geometry_msgs_AccelStamped.accel.linear.x * a_geometry_msgs_Accel.linear.x;

  return 0;
}
