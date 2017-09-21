#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


void a(geometry_msgs::Point m)
{
  m.x = m.y;
}

void b(geometry_msgs::Quaternion m)
{
  m.x = m.y;
}

int main(int argc, char **argv)
{
  float f = 10; 
  return 1;
  
}
