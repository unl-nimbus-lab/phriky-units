#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  tf2::Transform tf;
  tf2::Quaternion q;
  q = tf.getRotation();
  geometry_msgs::Twist tw;
  tw.angular.x = q.getZ() * tf.getRotation().getW();
}
