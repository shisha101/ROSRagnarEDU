
#include "ros_ragnar/ragnar_joint_feedback_message.h"
#include <sensor_msgs/JointState.h>

namespace ragnar_joint_feedback
{

// Request Message
Request::Request()
{

}

void Request::init()
{

}

bool Request::init(const shared_types::shared_real pose[6])
{

}

bool Request::load(industrial::byte_array::ByteArray* buffer)
{
  bool success = false;

  return success;
}

bool Request::unload(industrial::byte_array::ByteArray* buffer)
{
  bool success = false;
  return success;
}

unsigned int Request::byteLength() {
  6 * sizeof(industrial::shared_types::shared_real);
}


// Response Message
Response::Response()
{

}

Response::~Response()
{

}

bool Response::init(industrial::simple_message::SimpleMessage& msg)
{

}

void Response::init()
{

}

bool Response::load(industrial::byte_array::ByteArray* buffer)
{
  bool success = false;
  if(buffer->load(this->robot_id_))
  {
    if(buffer->load(this->valid_fields_))
    {
      if(buffer->load(this->time_))
      {
        if(buffer->load(this->joint_values_))
        {
          if(buffer->load(this->velocity_values_))
          {
            if(buffer->load(this->acceleration_values_))
            {
              success = true;
            }
            else
            {
              LOG_ERROR("Failed to load acceleration values");
            }
          }
          else
          {
            LOG_ERROR("Failed to load velocity values");
          }
        }
        else
        {
          LOG_ERROR("Failed to load joint values");
        }
      }
      else
      {
        LOG_ERROR("Failed to load time");
      }
    }
    else
    {
      LOG_ERROR("Failed to load valid fields");
    }
  }
  else
  {
    LOG_ERROR("Failed to load robot id");
  }
  return success;
}

bool Response::unload(industrial::byte_array::ByteArray* buffer)
{
  bool success = false;
  if(buffer->unload(this->acceleration_values_))
  {
    if(buffer->unload(this->velocity_values_))
    {
      if(buffer->unload(this->joint_values_))
      {
        if(buffer->unload(this->time_))
        {
          if(buffer->unload(this->valid_fields_))
          {
            if(buffer->unload(this->robot_id_))
            {
               success = true;
            }
            else
            {
              LOG_ERROR("Failed to unload robot id");
            }
          }
          else
          {
            LOG_ERROR("Failed to unload valid fields");
          }
        }
        else
        {
          LOG_ERROR("Failed to unload time");
        }
      }
      else
      {
        LOG_ERROR("Failed to unload joint values");
      }
    }
    else
    {
      LOG_ERROR("Failed to unload velocity values");
    }
  }
  else
  {
    LOG_ERROR("Failed to unload acceleration values");
  }
  return success;
}

unsigned int Response::byteLength() {
  15 * sizeof(industrial::shared_types::shared_real);
}

std::vector<float> Response::getJointValues()
{
  std::vector<float> array;
  for(int i = 0; i < joint_values_.getJointArraySize(); ++i)
  {
    industrial::shared_types::shared_real value = joint_values_.getJointValue(industrial::shared_types::shared_int(i));
    array.push_back(float(value));
  }
  return array;
}

std::vector<float> Response::getVelocityValues()
{
  std::vector<float> array;
  for(int i = 0; i < velocity_values_.getJointArraySize(); ++i)
  {
    industrial::shared_types::shared_real value = velocity_values_.getJointValue(industrial::shared_types::shared_int(i));
    array.push_back(float(value));
  }
  return array;
}

std::vector<float> Response::getAccelerationValues()
{
  std::vector<float> array;
  for(int i = 0; i < acceleration_values_.getJointArraySize(); ++i)
  {
    industrial::shared_types::shared_real value = acceleration_values_.getJointValue(industrial::shared_types::shared_int(i));
    array.push_back(float(value));
  }
  return array;
}

// Message handler
RagnarJointFeedbackHandler::RagnarJointFeedbackHandler()
{

}

// Callback for receiving message from Ragnar, do we need this?
bool RagnarJointFeedbackHandler::internalCB(simple_message::SimpleMessage &in)
{
  // TODO: add logic here for receiveing joint feedback and publishing to joint states
  ragnar_joint_feedback::Response joint_data;

  industrial::byte_array::ByteArray* array;
  *array = in.getData();
  joint_data.unload(array);

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  std::vector<float> values;

  values = joint_data.getJointValues();
  for(int i = 0; i < values.size(); ++i)
  {
    msg.position.push_back(values[i]);
  }

  values = joint_data.getVelocityValues();
  for(int i = 0; i < values.size(); ++i)
  {
    msg.velocity.push_back(values[i]);
  }

  values = joint_data.getAccelerationValues();
  for(int i = 0; i < values.size(); ++i)
  {
    msg.effort.push_back(values[i]);
  }

  joint_states_pub_->publish(msg);

  //simple_message::SimpleMessage reply;
  //this->getConnection()->sendMsg(reply);
  return true;
}

}// ragnar_joint_feedback namespace

