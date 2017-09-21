#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{

  geometry_msgs::Twist tw;

  tw.linear.x = 1.;  
  tw.linear.y = 1.;  
  tw.linear.y = 1.;  

  float f = (float) ((float)( tw.linear.x * 1.) + ((float) (tw.linear.y + 1.) + tw.linear.z));

}
