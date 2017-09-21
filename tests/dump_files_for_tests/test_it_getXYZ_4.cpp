#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv)
{
  tf2::Transform tf;
  geometry_msgs::Twist tw;
  tw.linear.x = tf.getRotation().getX();
  return 0;
}




