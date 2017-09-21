#include "geometry_msgs/Accel.h"

int main(int argc, char **argv)
{

geometry_msgs::Accel acc;

// DIV SAME UNITS
float y = 10;
float x = y  / acc.angular.z;

}
