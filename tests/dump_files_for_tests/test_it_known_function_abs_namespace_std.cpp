#include <ros/ros.h>
#include <math.h>

using namespace std;
int main(int argc, char **argv)
{
  ros::Duration d1(0.1);
  float s = abs(d1.toSec());
  float t = fabs(d1.toSec());

}
