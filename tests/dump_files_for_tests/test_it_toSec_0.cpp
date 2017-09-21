#include <ros/ros.h>

int main(int argc, char **argv)
{
  
  ros::Duration d1(0.1);
  float duration = ros::Time::now().toSec();
  ros::Time t1 = (ros::Time::now() + d1);
  float seconds = (ros::Time::now() + d1).toSec();

}
