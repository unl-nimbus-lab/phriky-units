#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv)
{
  tf2::Transform tf;
  tf2::Quaternion q;
  geometry_msgs::Twist tw;
  q = tf.getRotation();
  tw.linear.x = q.getX();
  return 0;
}




