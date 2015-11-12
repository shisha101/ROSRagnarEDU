#ifndef RAGNAR_JOINT_FEEDBACK_MESSAGE_H
#define RAGNAR_JOINT_FEEDBACK_MESSAGE_H

#include <simple_message/typed_message.h>     // TriggerRequest/TriggerResponse base classes
#include <simple_message/simple_message.h>    // SimpleMessage class
#include <simple_message/shared_types.h>      // SimpleMessage primitive types
#include <simple_message/log_wrapper.h>       // To aid in logging
#include <simple_message/message_handler.h>

#include <ros/ros.h>
#include <ros_ragnar/joint_data.h>

using namespace industrial;

namespace ragnar_joint_feedback
{

const int MSG_JOINT_FEEDBACK = 15;

class Request : public industrial::typed_message::TypedMessage
{
public:
  Request();
  ~Request(){}
  void init();
  bool init(const shared_types::shared_real pose[6]);

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  unsigned int byteLength();

private:
  shared_types::shared_real pose_[6];

};

class Response : public industrial::typed_message::TypedMessage
{
public:
  Response();
  ~Response();

  bool init(industrial::simple_message::SimpleMessage& msg);
  void init();

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  unsigned int byteLength();

  int getRobotID(){return (int)robot_id_;}
  int getValidFields(){return (int)valid_fields_;}
  float getTime(){return (float)time_;}
  std::vector<float> getJointValues();
  std::vector<float> getVelocityValues();
  std::vector<float> getAccelerationValues();

private:
  industrial::shared_types::shared_int robot_id_;
  industrial::shared_types::shared_int valid_fields_;
  industrial::shared_types::shared_real time_;

  joint_data::JointData joint_values_;
  joint_data::JointData velocity_values_;
  joint_data::JointData acceleration_values_;

};

class RagnarJointFeedbackHandler : public message_handler::MessageHandler
{

public:
  RagnarJointFeedbackHandler();
  ~RagnarJointFeedbackHandler(){}

  using message_handler::MessageHandler::init;
  bool internalCB(simple_message::SimpleMessage &in);

  bool setJointStatePublisher(ros::Publisher* pub){joint_states_pub_ = pub;}
private:
  ros::Publisher* joint_states_pub_;
};

}


#endif // RAGNAR_JOINT_FEEDBACK_MESSAGE_H
