#include "ros_ragnar/ragnar_message_manager.h"
#include "ros_ragnar/ragnar_position_message_handler.h"

bool RagnarMessageManager::init(const int& tcp_port)
{
  // make server connection on tcp port
  tcp_port_ = tcp_port;
  server_.init(tcp_port_);
  makeConnect();  // Make socket connection, loops indefinitely until connection is made

  // establish message manager connection and add handlers
  manager_.init(connection_);

  manager_.add(&position_handler_);

  return true;
}
