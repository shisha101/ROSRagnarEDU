
#include "ragnar_drivers/ragnar_joint_feedback_message.h"
#include <sensor_msgs/JointState.h>
#include <simple_message/joint_data.h>

namespace ragnar_drivers
{
using namespace industrial;

// Message handler
RagnarJointFeedbackHandler::RagnarJointFeedbackHandler(const std::vector<std::string> &joint_names)
  : joint_names_(joint_names)
{
  ROS_ASSERT(joint_names.size() == 4);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

// Callback for receiving message from Ragnar, do we need this?
bool RagnarJointFeedbackHandler::internalCB(simple_message::SimpleMessage& in)
{
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";

  msg.name = joint_names_;

  joint_feedback_message::JointFeedbackMessage joint_msg;
  joint_msg.init(in);

  // populate msg for publishing
  joint_data::JointData data;

  joint_msg.getPositions(data);
  for (int i = 3; i >= 0; --i)
  {
    msg.position.push_back((-data.getJoint(i) * M_PI / 180.0));
  }

  joint_msg.getVelocities(data);
  for (int i = 3; i >= 0; --i)
  {
    msg.velocity.push_back(-data.getJoint(i) * M_PI / 180.0);
  }

  joint_msg.getAccelerations(data);
  for (int i = 3; i >= 0; --i)
  {
    msg.effort.push_back(-data.getJoint(i) * M_PI / 180.0);
  }

  joint_states_pub_.publish(msg);

  return true;
}

} // ragnar_joint_feedback namespace
