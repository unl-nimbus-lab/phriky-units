#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

float g(int x){

geometry_msgs::Twist tw;
float y = tw.linear.x + tw.angular.x;

return y;

}
