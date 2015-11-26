#include <ros/ros.h>

#include "ragnar_state_publisher/ragnar_state_publisher.h"


static std::vector<std::string> getDefaultRagnarJointNames()
{
  std::vector<std::string> names;
  names.reserve(4);
  names.push_back("joint_1");
  names.push_back("joint_2");
  names.push_back("joint_3");
  names.push_back("joint_4");
  return names;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_state_publisher");
  ros::NodeHandle nh;

  std::vector<std::string> joint_names;
  if (!nh.getParam("controller_joint_names", joint_names))
  {
    joint_names = getDefaultRagnarJointNames();
    ROS_INFO("Ragnar State Publisher: loading default joint names [joint_1 ... joint_4]");
  }
  else
  {
    ROS_INFO("Ragnar State Publisher: loaded joint names from param server");
  }

  ragnar_state_publisher::RagnarStatePublisher pub ("joint_states", joint_names);

  ros::spin();
}
