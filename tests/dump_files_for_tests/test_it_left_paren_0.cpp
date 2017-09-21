#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{
  sensor_msgs::Range r;
  geometry_msgs::TwistStamped ts;
  ts.header.stamp.sec = 0.;
  ts.twist.linear.x = 10.;

  r.range = 0;

  float meters_type_0 = (ts.twist.linear.x * r.header.stamp.sec);
  float meters_type_1 = r.range + (ts.twist.linear.x * r.header.stamp.sec);
  float meters_type_2 = (ts.twist.linear.x * r.header.stamp.sec) + r.range;
  float meters_type_3 = (r.range + r.range) + (r.range + r.range);
  float meters_type_4 = ((r.range) + r.range); 
  float meters_type_5 = (r.range + (r.range)); 
  float meters_type_6 = ((((((r.range + r.range))))));
  float meters_type_7 = ((((((r.range + r.range))))+ r.range));

}
