
#include "ros_ragnar/ragnar_position_message_handler.h"

namespace ragnar_position_message
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

  return success;
}

bool Response::unload(industrial::byte_array::ByteArray* buffer)
{
  bool success = false;
  return success;
}

unsigned int Response::byteLength() {
  8 * sizeof(industrial::shared_types::shared_real);
}

// Message handler
RagnarPositionMessageHandler::RagnarPositionMessageHandler()
{

}

// Callback for receiving message from Ragnar, do we need this?
bool RagnarPositionMessageHandler::internalCB(simple_message::SimpleMessage &in)
{
  // TODO: add logic here for receiveing joint commands and moving robot
  simple_message::SimpleMessage reply;

  this->getConnection()->sendMsg(reply);

}

}
