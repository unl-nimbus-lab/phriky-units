#include "sensor_msgs/Range.h"


float unknown_f(float x)
{
  return 2. * x;
}

int main(int argc, char **argv)
{
  sensor_msgs::Range r;

  r.range = 0;
  
  float unknown_type_0 = unknown_f(r.range);  // TYPE SHOULD NOT PROPAGATE ACROSS (
  float unknown_type_1 = unknown_f(r.range + r.range);  // TYPE SHOULD NOT PROPAGATE ACROSS (
  float unknown_type_2 = unknown_f(r.range + (r.range + r.range));  // TYPE SHOULD NOT PROPAGATE ACROSS (

}
