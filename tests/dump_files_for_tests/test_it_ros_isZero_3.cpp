#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char **argv)
{
  geometry_msgs::TwistStamped tws;
  if(!tws.header.stamp.isZero())
    int t = 42;

}
