#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <math.h>

int main(int argc, char **argv)
{
	 float left_vel;
	 float right_vel;
  float maxSpeed_ = 1.0;
  geometry_msgs::Twist msg;

  left_vel = msg.linear.x - msg.angular.z;
  right_vel = msg.linear.x - msg.angular.z;

		double large_speed = std::fmax(std::abs(left_vel), std::abs(right_vel));

		if (large_speed > maxSpeed_) {
				left_vel *= maxSpeed_ / large_speed;
				right_vel *= maxSpeed_ / large_speed;
		}

}

//void cmdReceived(const geometry_msgs::Twist& msg) {
      ////vx = boost::algorithm::clamp(msg.linear.x, -1.0, 1.0);
      ////vtheta = boost::algorithm::clamp(msg.angular.z, -1.0, 1.0);
      ////left_vel = round(((vx - vtheta)+2)*slope-1000);
      ////right_vel = round(((-vx - vtheta)+2)*slope-1000);
      //left_vel = msg.linear.x - msg.angular.z;
      //right_vel = -msg.linear.x - msg.angular.z;
      //double large_speed = std::max(std::abs(left_vel), std::abs(right_vel));
      //if (large_speed > maxSpeed_) {
        //left_vel *= maxSpeed_ / large_speed;
        //right_vel *= maxSpeed_ / large_speed;
      //}
      //LEFT_vel = round(left_vel*1000);
      //RIGHT_vel = round(right_vel*1000);
      //string p = "MMW !M ";
      //p += to_string(LEFT_vel) + " " + to_string(RIGHT_vel);
      //int nLen = p.length();
      //drrobotMotionDriver_ -> sendCommand(p.c_str(), nLen); // Send command to robot
    //}

