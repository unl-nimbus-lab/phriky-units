#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{

  geometry_msgs::Twist tw;

  tw.linear.x = 42.0;  

  float f = (float) ( tw.linear.x * 1.618);
  f = ( tw.linear.x * 1.618);

}
