#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>


float f1(float r)
{
  sensor_msgs::Range b;
  b.header.stamp.sec = 42;
  return r / b.header.stamp.sec;
}

double v(void){
  return 1234.6789;
}


double d(float d){
  sensor_msgs::Range a_sensor_msgs_Range;
  geometry_msgs::TwistStamped ts;
  ts.twist.linear.x = 0;
  a_sensor_msgs_Range.range = 3.14;
  float y = 0;

  v();  // AVOID ANNOTATING v
  float x = f1(a_sensor_msgs_Range.range);  // GET RETURN UNITS FROM f1  DURING INIT
  y =  (1 * x)  + a_sensor_msgs_Range.range / d ;
  ts.twist.linear.x = y + (x * d);  // INCONSISTENCY IS HERE

}

int main(int argc, char **argv) {
  d(3.24);
  return 0;
}
