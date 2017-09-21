#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{

geometry_msgs::Twist t;
float f;
float z = 11.;

f = t.angular.x + (t.linear.x + z);  // UNITS SHOULD CAUSE ERROR BECAUSE + DOESN'T WEAKEN UNITS

  return 0;
}





