#include "geometry_msgs/Twist.h"
#include "math.h"

int main(int argc, char **argv)
{
  geometry_msgs::Twist t;
  float f;
  f = std::min(t.angular.x, t.linear.y);
  return 0;
}





