#include "geometry_msgs/Twist.h"
#include "math.h"

int main(int argc, char **argv)
{
  geometry_msgs::Twist t;
  float f;
  f = std::max(0., t.angular.x);
  return 0;
}





