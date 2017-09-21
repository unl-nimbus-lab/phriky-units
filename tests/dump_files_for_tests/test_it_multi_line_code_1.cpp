#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


float f(int x){
  return 3.14;
}

int main(int argc, char **argv)
{

  geometry_msgs::Twist t;

  float x = t.angular.z * f(10);
  float y  = std::sqrt( (t.linear.x * t.linear.x) +
          (t.linear.y * t.linear.y) +
          (x * x) );

}
