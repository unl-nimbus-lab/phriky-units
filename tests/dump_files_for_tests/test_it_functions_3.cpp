#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <ros/ros.h>

float f1(sensor_msgs::Range &r)
{
  float return_value = r.range + 3.14;
  return_value = f2(r.range) + 3.14;
  return return_value;
}

float f2(float r)
{
  float return_value = r*r;
  return return_value;
}

float f3(float r)
{
  ros::Time s;
  return r * ros::Time::now();
}

int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r = a_sensor_msgs_Range.range;

  float x = f1(a_sensor_msgs_Range);
  float y = f2(r);
  float z = f3(f2(r));

  return 0;
}
