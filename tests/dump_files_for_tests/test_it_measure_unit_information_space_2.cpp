#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  geometry_msgs::Twist tw;


  float pi;
  pi = 3.14;
  pi = tw.linear.x;
  tw.linear.y = tw.linear.x;
  tw.linear.z = tw.linear.x;
  a_sensor_msgs_Range.range = pi + 3.3 + tw.linear.x;
  return  a_sensor_msgs_Range.range;
}
