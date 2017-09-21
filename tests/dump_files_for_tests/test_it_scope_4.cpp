#include "sensor_msgs/Range.h"

float a;

void f2(void)
{
  float a_ = a;   // DOES THIS GET METERS?
}

int main(int argc, char **argv)
{
  
  sensor_msgs::Range r;
  r.range = 0;
  a = r.range;

  return 0;
}
