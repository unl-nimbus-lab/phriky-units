#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
  tf2::Transform tf;
  tf2::Quaternion q;
  q = tf.getRotation();
  float f = q.getZ() * tf.getRotation().getW();
}
