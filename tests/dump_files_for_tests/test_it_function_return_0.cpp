#include <sensor_msgs/Range.h>

float f1(void)
{
  sensor_msgs::Range b;
  b.range = 42.0;
  return b.range;
}

int main(int argc, char **argv)
{
  float x = f1();

  return 0;
}
