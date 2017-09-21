#include "geometry_msgs/Accel.h"

int main(int argc, char **argv)
{

geometry_msgs::Accel acc;

// DIV SAME UNITS
float x = fabs(acc.linear.y) / (acc.angular.z * acc.angular.z);

}
