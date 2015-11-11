#include <ros/ros.h>

#include <simple_message/simple_message.h>
#include <simple_message/socket/tcp_client.h>

#include <ros_ragnar/ragnar_message_manager.h>

int DEFAULT_TCP_PORT = 11002;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_feedback_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int tcp_port;
  std::string ip_addr;
  pnh.param<int>("tcp_port", tcp_port, DEFAULT_TCP_PORT);
  pnh.param<std::string>("ip", ip_addr, "");

  industrial::tcp_client::TcpClient client;

  if(ip_addr)
  {
    client.init(ip_addr, tcp_port);
  }


  while(!client.isConnected() && ros::ok())
  {
    client.makeConnect();
  }

  while(ros::ok())
  {
    client.receiveMsg();
  }

  ros::waitForShutdown();
  return 0;
}
