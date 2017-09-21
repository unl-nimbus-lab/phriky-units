#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
  sensor_msgs::Range a_range;
  geometry_msgs::TwistStamped a_twist;
  a_range.range = 1;
  a_twist.twist.linear.x = 1;

  if (a_range.range > a_twist.twist.linear.x)
  {
    a_range.range = 2; 
  }
  return 0;
}
