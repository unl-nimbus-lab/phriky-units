#include <sensor_msgs/Range.h>
#include <math.h>
using namespace std;
int main(int argc, char **argv)
{
  sensor_msgs::Range r;
  r.range = 0;

  float f = pow(42.0 * r.range * r.range, 2.0f);  // 42 ADDS UNCERTAINTY
}
