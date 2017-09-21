#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv)
{
  sensor_msgs::LaserScan ls;

  if (!ls.ranges.empty())
    int x = ls.ranges.size();
}
