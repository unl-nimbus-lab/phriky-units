#ifndef METABLOCK_H
#define METABLOCK_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/CollisionObject.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

namespace moveit_simple_actions
{

class MetaBlock
{
public:
  MetaBlock(const std::string name,
            const geometry_msgs::Pose start_pose,
            const uint shapeType,
            const double size_x,
            const double size_y,
            const double size_z,
            ros::Time timestamp=ros::Time::now());

  MetaBlock(const std::string name,
            const geometry_msgs::Pose start_pose,
            const shape_msgs::Mesh mesh,
            const object_recognition_msgs::ObjectType type,
            ros::Time timestamp=ros::Time::now());

  //update the object pose
  void updatePose(const geometry_msgs::Pose &start_pose);

  //update the object pose only visually without updating start_pose
  void updatePoseVis(const geometry_msgs::Pose &start_pose);

  moveit_msgs::CollisionObject wrapToCollisionObject(const std::vector <shape_msgs::Mesh> &meshes);

  //object name
  std::string name_;

  //the current position
  geometry_msgs::Pose start_pose_;

  //the goal position
  geometry_msgs::Pose goal_pose_;

  //corresponding collision object
  moveit_msgs::CollisionObject collObj_;

  //x dimenssion
  double size_x_;
  //y dimenssion
  double size_y_;
  //z dimenssion
  double size_z_;

  //timestamp of creation
  ros::Time timestamp_;

  //corresponding object type in DB
  object_recognition_msgs::ObjectType type_;

protected:
  std::string base_frame_;
  shape_msgs::SolidPrimitive shape_;
  shape_msgs::Mesh mesh_;
};
}

#endif // METABLOCK_H
