#include <sensor_msgs/Range.h>

double d(float d){
  sensor_msgs::Range a_sensor_msgs_Range;
  a_sensor_msgs_Range.range = 3.14;
  ts.twist.linear.x = a_sensor_msgs_Range.range / (d + d + d + d + d) ;// TESTS NUMBER OF s
}

int main(int argc, char **argv) {
  return 0;
}
