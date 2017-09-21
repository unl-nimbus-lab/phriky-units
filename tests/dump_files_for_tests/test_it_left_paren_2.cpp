#include "ros.h"


int main(int argc, char **argv)
{
  
  ros::Time t1 = ros::Time::now();
  ros::Time t2 = 0.;
  float duration = (t1 - t2).toSec();

}
