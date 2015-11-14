#ifndef RAGNAR_JOINT_FEEDBACK_MESSAGE_H
#define RAGNAR_JOINT_FEEDBACK_MESSAGE_H

#include <simple_message/typed_message.h> // TriggerRequest/TriggerResponse base classes
#include <simple_message/simple_message.h> // SimpleMessage class
#include <simple_message/shared_types.h>   // SimpleMessage primitive types
#include <simple_message/log_wrapper.h>    // To aid in logging
#include <simple_message/message_handler.h>

#include <simple_message/joint_feedback.h>
#include <simple_message/messages/joint_feedback_message.h>

#include <ros/ros.h>

namespace ros_ragnar
{

class RagnarJointFeedbackHandler
    : public industrial::message_handler::MessageHandler
{

public:
  RagnarJointFeedbackHandler();
  ~RagnarJointFeedbackHandler() {}

  using industrial::message_handler::MessageHandler::init;

  bool internalCB(industrial::simple_message::SimpleMessage& in);

  void setCBConnection(
      industrial::smpl_msg_connection::SmplMsgConnection* connection)
  {
    init(industrial::simple_message::StandardMsgTypes::JOINT_FEEDBACK,
         connection);
    ROS_INFO("RagnarJointFeedbackHandler initialized");
  }

  bool setNodeHandle(ros::NodeHandle& nh) { nh_ = nh; }

private:
  ros::NodeHandle nh_;
  ros::Publisher joint_states_pub_;
};
}

#endif // RAGNAR_JOINT_FEEDBACK_MESSAGE_H
