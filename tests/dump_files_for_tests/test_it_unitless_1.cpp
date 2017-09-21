#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
  sensor_msgs::Range a_range;
  sensor_msgs::Range b_range;
  a_range.range = 1;
  b_range.range = 71;

  float z = a_range.range / b_range.range;

  return 0;
}
