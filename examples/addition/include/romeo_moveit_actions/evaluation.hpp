#ifndef EVALUATION_HPP
#define EVALUATION_HPP

#include <ros/ros.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"

namespace moveit_simple_actions
{

class Evaluation
{
public:

  Evaluation(const bool &verbose, const std::string &base_frame);

  void init(const double &test_step,
            const double &block_size_x,
            const double &block_size_y,
            const double floor_to_base_height,
            const double &x_min,
            const double &x_max,
            const double &y_min,
            const double &y_max,
            const double &z_min,
            const double &z_max);

  void testReach(ros::NodeHandle &nh,
                 ros::Publisher *pub_obj_pose,
                 ros::Publisher *pub_obj_poses,
                 ros::Publisher *pub_obj_moveit,
                 moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                 Action *action_left,
                 Action *action_right,
                 std::vector<MetaBlock> *blocks_surfaces,
                 const bool pickVsReach,
                 const bool test_poses_rnd=false);

  void printStat();

protected:
  geometry_msgs::PoseArray generatePosesGrid(std::vector<MetaBlock> &blocks_test);

  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr,
                                            std::vector<MetaBlock> &blocks_test);

  int testReachWithGenSingleHand(Action *action,
                                 std::vector<MetaBlock> *blocks_surfaces,
                                 ros::Publisher *pub_obj_pose,
                                 ros::Publisher *pub_obj_poses,
                                 ros::Publisher *pub_obj_moveit,
                                 moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                                 const bool pickVsReach,
                                 const int attempts_nbr,
                                 const double planning_time,
                                 geometry_msgs::PoseArray &msg_poses_validated);

  bool verbose_;

  //robot's base_frame
  std::string base_frame_;

  //the interval to test the working space
  double test_step_;

  //the size of a default object
  double block_size_x_;
  double block_size_y_;

  //the shift of the robot's base to teh floor
  double floor_to_base_height_;

  //the working space of the robot
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  geometry_msgs::Pose pose_zero_;

  //successfully reached positions
  std::vector <geometry_msgs::Pose> stat_poses_success_;

};
}
#endif // EVALUATION_HPP
