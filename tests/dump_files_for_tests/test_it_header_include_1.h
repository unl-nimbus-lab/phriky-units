#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

float f(int x){

geometry_msgs::Twist tw;
return tw.linear.x + tw.angular.x;

}
