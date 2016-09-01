#ifndef TOOLSFORACTION_HPP
#define TOOLSFORACTION_HPP

// ROS
#include <ros/ros.h>

namespace moveit_simple_actions
{

template<typename T>
void waitForAction(const T &action, const ros::NodeHandle &node_handle,
                   const ros::Duration &wait_for_server, const std::string &name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    while (node_handle.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    ros::Time final_time = ros::Time::now() + wait_for_server;
    while (node_handle.ok() && !action->isServerConnected() && final_time > ros::Time::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to move_group action server within allotted time (2)");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
}
}

#endif // TOOLSFORACTION_HPP
