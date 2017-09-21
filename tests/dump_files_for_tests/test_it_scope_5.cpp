#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"

float a;

void f1(void)
{
  sensor_msgs::Range r;
  r.range = 0;
  a = r.range;
}

void f2(void)
{
  
  f1();
  geometry_msgs::TwistStamped a_;
  a_.twist.linear.x = a;  // METERS?
}


int main(int argc, char **argv)
{
  a = 0;
  f2();
  return 0;
}
