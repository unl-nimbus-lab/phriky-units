#include <ros/ros.h>
#include <math.h>

int main(int argc, char **argv)
{
  float f = std::atan(10); // f SHOULD GET RADIANS
  float z = 3.14;  // HAND ANNOTATION FOR DIMENSIONLESS
  float x = z * f;
}
