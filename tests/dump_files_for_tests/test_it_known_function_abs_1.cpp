#include <ros/ros.h>
#include <math.h>

int main(int argc, char **argv)
{
  
  ros::Duration d1(0.1);
  float s = std::abs(d1.toSec());
  float t = std::fabs(d1.toSec());
  int error [3];

  for(int i=0; i<3; i++){error[i]=0;}

  if (fabs(error[0]) > fabs(error[1]) && fabs(error[0]) > fabs(error[2]))
  {
    t = 42.0;
  }

  ros::Time my_time = ros::Time::now();
  if (std::abs((my_time  - d1).toSec()) > 37.2){
    t = 1.618033;
  }


}
