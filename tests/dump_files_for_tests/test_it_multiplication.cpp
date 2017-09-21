#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/AccelWithCovariance.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "geometry_msgs/Inertia.h"
#include "geometry_msgs/InertiaStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Illuminance.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserEcho.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Temperature.h"
#include "stereo_msgs/DisparityImage.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{

geometry_msgs::Accel a_geometry_msgs_Accel;
geometry_msgs::AccelStamped a_geometry_msgs_AccelStamped;
geometry_msgs::AccelWithCovariance a_geometry_msgs_AccelWithCovariance;
geometry_msgs::AccelWithCovarianceStamped a_geometry_msgs_AccelWithCovarianceStamped;
geometry_msgs::Inertia a_geometry_msgs_Inertia;
geometry_msgs::InertiaStamped a_geometry_msgs_InertiaStamped;
geometry_msgs::Point a_geometry_msgs_Point;
geometry_msgs::Point32 a_geometry_msgs_Point32;
geometry_msgs::PointStamped a_geometry_msgs_PointStamped;
geometry_msgs::Polygon a_geometry_msgs_Polygon;
geometry_msgs::PolygonStamped a_geometry_msgs_PolygonStamped;
geometry_msgs::Pose a_geometry_msgs_Pose;
geometry_msgs::Pose2D a_geometry_msgs_Pose2D;
geometry_msgs::PoseStamped a_geometry_msgs_PoseStamped;
geometry_msgs::PoseWithCovariance a_geometry_msgs_PoseWithCovariance;
geometry_msgs::PoseWithCovarianceStamped a_geometry_msgs_PoseWithCovarianceStamped;
geometry_msgs::Quaternion a_geometry_msgs_Quaternion;
geometry_msgs::QuaternionStamped a_geometry_msgs_QuaternionStamped;
geometry_msgs::Transform a_geometry_msgs_Transform;
geometry_msgs::TransformStamped a_geometry_msgs_TransformStamped;
geometry_msgs::Twist a_geometry_msgs_Twist;
geometry_msgs::TwistStamped a_geometry_msgs_TwistStamped;
geometry_msgs::TwistWithCovariance a_geometry_msgs_TwistWithCovariance;
geometry_msgs::TwistWithCovarianceStamped a_geometry_msgs_TwistWithCovarianceStamped;
geometry_msgs::Wrench a_geometry_msgs_Wrench;
geometry_msgs::WrenchStamped a_geometry_msgs_WrenchStamped;
nav_msgs::GridCells a_nav_msgs_GridCells;
nav_msgs::MapMetaData a_nav_msgs_MapMetaData;
sensor_msgs::FluidPressure a_sensor_msgs_FluidPressure;
sensor_msgs::Illuminance a_sensor_msgs_Illuminance;
sensor_msgs::Imu a_sensor_msgs_Imu;
sensor_msgs::JointState a_sensor_msgs_JointState;
sensor_msgs::LaserEcho a_sensor_msgs_LaserEcho;
sensor_msgs::LaserScan a_sensor_msgs_LaserScan;
sensor_msgs::MagneticField a_sensor_msgs_MagneticField;
sensor_msgs::MultiDOFJointState a_sensor_msgs_MultiDOFJointState;
sensor_msgs::MultiEchoLaserScan a_sensor_msgs_MultiEchoLaserScan;
sensor_msgs::NavSatFix a_sensor_msgs_NavSatFix;
sensor_msgs::PointCloud a_sensor_msgs_PointCloud;
sensor_msgs::PointCloud2 a_sensor_msgs_PointCloud2;
sensor_msgs::Range a_sensor_msgs_Range;
sensor_msgs::Temperature a_sensor_msgs_Temperature;
stereo_msgs::DisparityImage a_stereo_msgs_DisparityImage;

a_geometry_msgs_Accel.linear.x = 0;
a_geometry_msgs_Accel.angular.x = 0;

a_geometry_msgs_AccelStamped.header.stamp.sec = 0;
a_geometry_msgs_AccelStamped.accel.linear.x = 0;
a_geometry_msgs_AccelStamped.accel.angular.x = 0;

a_geometry_msgs_AccelWithCovariance.accel.linear.x = 0;
a_geometry_msgs_AccelWithCovariance.accel.angular.x = 0;

a_geometry_msgs_AccelWithCovarianceStamped.header.stamp.sec= 0;
a_geometry_msgs_AccelWithCovarianceStamped.accel.accel.linear.x = 0;
a_geometry_msgs_AccelWithCovarianceStamped.accel.accel.angular.x = 0;

a_geometry_msgs_Inertia.ixz = 0;
a_geometry_msgs_Inertia.ixx = 0;
a_geometry_msgs_Inertia.ixy = 0;
a_geometry_msgs_Inertia.com.x = 0;
a_geometry_msgs_Inertia.izz = 0;
a_geometry_msgs_Inertia.iyy = 0;
a_geometry_msgs_Inertia.m = 0;
a_geometry_msgs_Inertia.iyz = 0;

a_geometry_msgs_InertiaStamped.inertia.ixz = 0;
a_geometry_msgs_InertiaStamped.inertia.ixx = 0;
a_geometry_msgs_InertiaStamped.inertia.ixy = 0;
a_geometry_msgs_InertiaStamped.inertia.com.x = 0;
a_geometry_msgs_InertiaStamped.inertia.izz = 0;
a_geometry_msgs_InertiaStamped.inertia.iyy = 0;
a_geometry_msgs_InertiaStamped.inertia.m = 0;
a_geometry_msgs_InertiaStamped.inertia.iyz = 0;

a_geometry_msgs_Point.y = 0;
a_geometry_msgs_Point.x = 0;
a_geometry_msgs_Point.z = 0;

a_geometry_msgs_Point32.y = 0;
a_geometry_msgs_Point32.x = 0;
a_geometry_msgs_Point32.z = 0;

a_geometry_msgs_PointStamped.point.y = 0;
a_geometry_msgs_PointStamped.point.x = 0;
a_geometry_msgs_PointStamped.point.z = 0;
a_geometry_msgs_PointStamped.header.stamp.sec = 0;

//geometry::msgs myPoints = []
//a_geometry_msgs_Polygon.points = 0;

a_geometry_msgs_PolygonStamped.header.stamp.sec = 0;
//a_geometry_msgs_PolygonStamped.polygon.points = 0;

a_geometry_msgs_Pose.position.x = 0;
a_geometry_msgs_Pose.orientation.x = 0;

a_geometry_msgs_Pose2D.y = 0;
a_geometry_msgs_Pose2D.x = 0;
a_geometry_msgs_Pose2D.theta = 0;

a_geometry_msgs_PoseStamped.header.stamp.sec = 0;
a_geometry_msgs_PoseStamped.pose.position.x = 0;
a_geometry_msgs_PoseStamped.pose.orientation.x = 0;

a_geometry_msgs_PoseWithCovariance.pose.position.x = 0;
a_geometry_msgs_PoseWithCovariance.pose.orientation.x = 0;

a_geometry_msgs_PoseWithCovarianceStamped.header.stamp.sec = 0;
a_geometry_msgs_PoseWithCovarianceStamped.pose.pose.position.x = 0;
a_geometry_msgs_PoseWithCovarianceStamped.pose.pose.orientation.x = 0;

a_geometry_msgs_Quaternion.y = 0;
a_geometry_msgs_Quaternion.x = 0;
a_geometry_msgs_Quaternion.z = 0;
a_geometry_msgs_Quaternion.w = 0;

a_geometry_msgs_QuaternionStamped.quaternion.y = 0;
a_geometry_msgs_QuaternionStamped.quaternion.x = 0;
a_geometry_msgs_QuaternionStamped.quaternion.z = 0;
a_geometry_msgs_QuaternionStamped.quaternion.w = 0;
a_geometry_msgs_QuaternionStamped.header.stamp.sec = 0;

a_geometry_msgs_Transform.translation.x = 0;
a_geometry_msgs_Transform.rotation.x = 0;

a_geometry_msgs_TransformStamped.header.stamp.sec = 0;
a_geometry_msgs_TransformStamped.transform.translation.x = 0;
a_geometry_msgs_TransformStamped.transform.rotation.x = 0;

a_geometry_msgs_Twist.linear.x = 0;
a_geometry_msgs_Twist.angular.x = 0;

a_geometry_msgs_TwistStamped.twist.linear.x = 0;
a_geometry_msgs_TwistStamped.twist.angular.x = 0;
a_geometry_msgs_TwistStamped.header.stamp.sec = 0;

a_geometry_msgs_TwistWithCovariance.twist.linear.x = 0;
a_geometry_msgs_TwistWithCovariance.twist.angular.x = 0;

a_geometry_msgs_TwistWithCovarianceStamped.header.stamp.sec = 0;
a_geometry_msgs_TwistWithCovarianceStamped.twist.twist.linear.x = 0;
a_geometry_msgs_TwistWithCovarianceStamped.twist.twist.angular.x = 0;

a_geometry_msgs_Wrench.force.x = 0;
a_geometry_msgs_Wrench.torque.x = 0;

a_geometry_msgs_WrenchStamped.header.stamp.sec = 0;
a_geometry_msgs_WrenchStamped.wrench.force.x = 0;
a_geometry_msgs_WrenchStamped.wrench.torque.x = 0;

a_nav_msgs_GridCells.cell_height = 0;
a_nav_msgs_GridCells.cell_width = 0;

a_nav_msgs_MapMetaData.map_load_time.sec = 0;

a_sensor_msgs_FluidPressure.fluid_pressure = 0;
a_sensor_msgs_FluidPressure.variance = 0;
a_sensor_msgs_FluidPressure.header.stamp.sec = 0;

a_sensor_msgs_Illuminance.illuminance = 0;
a_sensor_msgs_Illuminance.variance = 0;
a_sensor_msgs_Illuminance.header.stamp.sec = 0;

//a_sensor_msgs_Imu.orientation_covariance = 0;  //todo covariance
a_sensor_msgs_Imu.orientation.x = 0;
//a_sensor_msgs_Imu.angular_velocity_covariance = 0;
a_sensor_msgs_Imu.header.stamp.sec = 0;
//a_sensor_msgs_Imu.linear_acceleration_covariance = 0;
a_sensor_msgs_Imu.linear_acceleration.x = 0;
a_sensor_msgs_Imu.angular_velocity.x = 0;

std_msgs::Float64MultiArray pos;
a_sensor_msgs_JointState.position = pos.data;
std_msgs::Float64MultiArray eff;
a_sensor_msgs_JointState.effort = eff.data;
std_msgs::Float64MultiArray vel;
a_sensor_msgs_JointState.velocity = vel.data;

std_msgs::Float32MultiArray echoes;
a_sensor_msgs_LaserEcho.echoes = echoes.data;

std_msgs::Float32MultiArray ran;
a_sensor_msgs_LaserScan.ranges = ran.data;
a_sensor_msgs_LaserScan.angle_min = 0;
a_sensor_msgs_LaserScan.header.stamp.sec = 0;
a_sensor_msgs_LaserScan.time_increment = 0;
a_sensor_msgs_LaserScan.scan_time = 0;
a_sensor_msgs_LaserScan.range_max = 0;
a_sensor_msgs_LaserScan.range_min = 0;
a_sensor_msgs_LaserScan.angle_increment = 0;
a_sensor_msgs_LaserScan.angle_max = 0;


boost::array<float, 9> mag_cov_data;
a_sensor_msgs_MagneticField.magnetic_field_covariance = mag_cov_data;
a_sensor_msgs_MagneticField.magnetic_field.x = 0;
a_sensor_msgs_MagneticField.header.stamp.sec = 0;

//todo: get these array's working
//a_sensor_msgs_MultiDOFJointState.twist = twist_array;
a_sensor_msgs_MultiDOFJointState.header.stamp.sec = 0;
//a_sensor_msgs_MultiDOFJointState.wrench = 0;
//a_sensor_msgs_MultiDOFJointState.transforms = 0;

//std_msgs::Float32MultiArray multi_ran;
//a_sensor_msgs_MultiEchoLaserScan.ranges = multi_ran;  //todo: multiple ranges
a_sensor_msgs_MultiEchoLaserScan.angle_min = 0;
a_sensor_msgs_MultiEchoLaserScan.header.stamp.sec = 0;
a_sensor_msgs_MultiEchoLaserScan.time_increment = 0;
a_sensor_msgs_MultiEchoLaserScan.scan_time = 0;
a_sensor_msgs_MultiEchoLaserScan.range_max = 0;
a_sensor_msgs_MultiEchoLaserScan.range_min = 0;
a_sensor_msgs_MultiEchoLaserScan.angle_increment = 0;
a_sensor_msgs_MultiEchoLaserScan.angle_max = 0;


a_sensor_msgs_NavSatFix.latitude = 0;
boost::array<float, 9> nav_sat_cov;
a_sensor_msgs_NavSatFix.position_covariance = nav_sat_cov;
a_sensor_msgs_NavSatFix.altitude = 0;
a_sensor_msgs_NavSatFix.longitude = 0;
a_sensor_msgs_NavSatFix.header.stamp.sec = 0;

a_sensor_msgs_PointCloud.header.stamp.sec = 0;
//todo: point32 points
//a_sensor_msgs_PointCloud.points = 0;

a_sensor_msgs_PointCloud2.header.stamp.sec = 0;
//a_sensor_msgs_PointCloud2.points = 0;

a_sensor_msgs_Range.header.stamp.sec = 0;
a_sensor_msgs_Range.field_of_view = 0;
a_sensor_msgs_Range.range = 0;
a_sensor_msgs_Range.min_range = 0;
a_sensor_msgs_Range.max_range = 0;

a_sensor_msgs_Temperature.header.stamp.sec = 0;
a_sensor_msgs_Temperature.temperature = 0;

a_stereo_msgs_DisparityImage.delta_d = 0;
a_stereo_msgs_DisparityImage.max_disparity = 0;
a_stereo_msgs_DisparityImage.T = 0;
a_stereo_msgs_DisparityImage.min_disparity = 0;



// MULT SAME UNITS
a_geometry_msgs_Accel.linear.x = a_geometry_msgs_AccelStamped.accel.linear.x * a_geometry_msgs_Accel.linear.x;
a_geometry_msgs_Accel.linear.x *= a_geometry_msgs_AccelStamped.accel.linear.x;

// DIV SAME UNITS
a_geometry_msgs_Accel.linear.x = a_geometry_msgs_AccelStamped.accel.linear.x / a_geometry_msgs_Accel.linear.x;
a_geometry_msgs_Accel.linear.x /= a_geometry_msgs_AccelStamped.accel.linear.x;


// MULT DIFFERENT UNITS
a_geometry_msgs_Accel.angular.x = a_geometry_msgs_AccelStamped.accel.linear.x * a_geometry_msgs_Accel.angular.x;
a_geometry_msgs_Accel.angular.x *= a_geometry_msgs_AccelStamped.accel.linear.x;


// DIV DIFFERENT UNITS
a_geometry_msgs_Accel.angular.x = a_geometry_msgs_AccelStamped.accel.linear.x / a_geometry_msgs_Accel.angular.x;
a_geometry_msgs_Accel.angular.x /= a_geometry_msgs_AccelStamped.accel.linear.x;



  return 0;
}
