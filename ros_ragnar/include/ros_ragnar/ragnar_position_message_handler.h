#ifndef RAGNAR_POSITION_MESSAGE_HANDLER_H
#define RAGNAR_POSITION_MESSAGE_HANDLER_H

#include <simple_message/typed_message.h>     // TriggerRequest/TriggerResponse base classes
#include <simple_message/simple_message.h>    // SimpleMessage class
#include <simple_message/shared_types.h>      // SimpleMessage primitive types
#include <simple_message/message_handler.h>

using namespace industrial;

namespace ragnar_position_message
{

const int POSITION_MESSAGE_TYPE = 65001;

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

private:
  shared_types::shared_int result_;

};

class RagnarPositionMessageHandler : public message_handler::MessageHandler
{

public:
  RagnarPositionMessageHandler();
  ~RagnarPositionMessageHandler(){}

  using message_handler::MessageHandler::init;
  bool internalCB(simple_message::SimpleMessage &in);
  bool sendPositionCommand(simple_message::SimpleMessage &in);

};

}
#endif // RAGNAR_POSITION_MESSAGE_HANDLER_H
