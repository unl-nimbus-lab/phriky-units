#include <sensor_msgs/Range.h>
#include <ros/ros.h>

int f1(sensor_msgs::Range r)
{
  return (int) (r.range + 1.618); 
}

int main(int argc, char **argv)
{
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 0;

  // SCOPE
  float r1= (float) f1(a_sensor_msgs_Range);  // SHOULD GET METERS

}
