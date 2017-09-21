#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{

geometry_msgs::Twist t;
float f;
float z = 11.;

f = t.angular.x + (t.linear.x * z);  // z MIGHT MAKE THE UNITS OK, BUT WE DON'T KNOW

  return 0;
}





