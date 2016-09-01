#ifndef TOOLSOBJ_H
#define TOOLSOBJ_H

namespace moveit_simple_actions
{

void setPose(geometry_msgs::Pose *pose, const double &x, const double &y, const double &z,
                            const double &ox, const double &oy, const double &oz, const double &ow)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
  pose->orientation.x = ox;
  pose->orientation.y = oy;
  pose->orientation.z = oz;
  pose->orientation.w = ow;
}

void setPose(geometry_msgs::Pose *pose, const double &x, const double &y, const double &z)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
}

int findObj(const std::vector<MetaBlock> &blocks, const std::string name)
{
  int idx = -1;
  for (int i=0; i<blocks.size(); ++i)
    if (blocks[i].name_ == name){
      idx = i;
      return idx;
    }
  return idx;
}

}

#endif // TOOLSOBJ_H
