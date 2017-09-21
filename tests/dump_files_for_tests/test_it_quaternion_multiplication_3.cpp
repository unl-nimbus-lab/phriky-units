#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
  tf2::Transform tf;
  float f = 42.0 * tf.getRotation().getW();
}
