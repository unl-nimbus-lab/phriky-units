#include "geometry_msgs/Accel.h"

#ifndef MY_ROS_DEF_
#define MY_ROS_DEF_

int f1(void){
return 0;
}
#endif 

int main(int argc, char **argv)
{

geometry_msgs::Accel a_geometry_msgs_Accel;
a_geometry_msgs_Accel.linear.x = 0;

// MULT SAME UNITS
float scale = a_geometry_msgs_Accel.linear.x;
a_geometry_msgs_Accel.linear.x *= scale;


  return 0;
}
