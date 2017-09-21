#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  geometry_msgs::Twist tw;

  float pi;
  float x;
  pi = 3.14;
  x  = tw.linear.x * pi; // UNKNOWN
  return  x;  // weak unknown
}
