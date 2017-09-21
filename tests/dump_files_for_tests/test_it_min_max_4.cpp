#include "geometry_msgs/Twist.h"
#include "math.h"

int main(int argc, char **argv)
{
  using std::min;
  geometry_msgs::Twist t;
  float f;
  f = min(t.angular.x, t.linear.y);
  return 0;
}





