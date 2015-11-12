#include <ros/ros.h>

#include <simple_message/simple_message.h>
#include <simple_message/socket/tcp_client.h>

#include <industrial_robot_client/robot_state_interface.h>

#include <ros_ragnar/ragnar_message_manager.h>
#include <ros_ragnar/ragnar_joint_feedback_message.h>

int DEFAULT_TCP_PORT = 11002;

class RagnarClient : public industrial_robot_client::robot_state_interface::RobotStateInterface
{

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_feedback_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int tcp_port;
  std::string ip_addr;
  pnh.param<int>("tcp_port", tcp_port, DEFAULT_TCP_PORT);
  pnh.param<std::string>("ip", ip_addr, "");

  //industrial::tcp_client::TcpClient client;
  RagnarClient client;

  ragnar_joint_feedback::RagnarJointFeedbackHandler feedback_handler;

  if(!ip_addr.empty())
  {
    //client.init((char*)ip_addr.c_str(), tcp_port);
    std::vector<std::string> joint_names;
    joint_names.push_back("J1");
    joint_names.push_back("J2");
    joint_names.push_back("J3");
    joint_names.push_back("J4");
    client.init(ip_addr, tcp_port);
    client.init(client.get_connection(),joint_names);
  }

  client.add_handler(&feedback_handler);

  client.run();

  while(ros::ok())
  {  
    //client.receiveMsg();
  }

  ros::waitForShutdown();
  return 0;
}
