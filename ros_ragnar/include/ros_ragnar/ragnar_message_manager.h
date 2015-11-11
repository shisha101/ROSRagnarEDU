#ifndef RAGNAR_MESSAGE_MANAGER_H
#define RAGNAR_MESSAGE_MANAGER_H

#include <ros/ros.h>
#include <simple_message/simple_message.h>                  // Simple message class
#include <simple_message/socket/tcp_server.h>               // SimpleMessage communications

//#include <simple_message/simple_message.h>                  // Simple message class
//#include <simple_message/message_handler.h>
#include <simple_message/message_manager.h>
#include <simple_message/smpl_msg_connection.h>

#include <ros_ragnar/ragnar_position_message_handler.h>

using namespace industrial;

class RagnarMessageManager : public smpl_msg_connection::SmplMsgConnection
{
private:
  ragnar_position_message::RagnarPositionMessageHandler position_handler_;

  bool sendBytes(byte_array::ByteArray &buffer){}
  bool receiveBytes(byte_array::ByteArray &buffer, shared_types::shared_int num_bytes){}

public:
  int tcp_port_;
  bool is_connected_;
  industrial::tcp_server::TcpServer server_;

  message_manager::MessageManager manager_;
  //industrial::message_handler::MessageHandler handler_;
  smpl_msg_connection::SmplMsgConnection* connection_;

  bool sendMsg(simple_message::SimpleMessage &message){server_.sendMsg(message);}
  bool receiveMsg(simple_message::SimpleMessage &message){}

  bool init(const int& tcp_port);

  bool makeConnect()
  {
      // Little message to know she's started
      ROS_INFO_STREAM("Starting pick position listener");

      // Create a TCP server connection on TCP_PORT (see common.h)
      server_.init(tcp_port_);

      // While server is not connected (while loop):
      // * Print out a helpful info message
      // * Try to connect
      // * Sleep for half a second (use ROS library call: ros::Duration(0.5).sleep())
      while(!server_.isConnected())
      {
        ROS_INFO_STREAM("Trying to connect to client");
        server_.makeConnect();
        ros::Duration(0.5).sleep();
      }
      is_connected_ = true;
      return true;
  }

  bool isConnected()
  {
      return is_connected_;
  }


};

#endif // RAGNAR_MESSAGE_MANAGER_H
