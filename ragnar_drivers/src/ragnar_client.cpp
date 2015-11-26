#include <ros/ros.h>

#include <simple_message/simple_message.h>
#include <simple_message/socket/tcp_client.h>

#include <industrial_robot_client/robot_state_interface.h>

#include <ragnar_drivers/ragnar_joint_feedback_message.h>
#include <simple_message/joint_feedback.h>

int DEFAULT_TCP_PORT = 11002;

using industrial_robot_client::robot_state_interface::RobotStateInterface;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_feedback_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int tcp_port;
  std::string ip_addr;
  pnh.param<int>("tcp_port", tcp_port, DEFAULT_TCP_PORT);
  pnh.param<std::string>("ip", ip_addr, "");

  RobotStateInterface client;


  if (!ip_addr.empty())
  {
    // make socket connection
    client.init(ip_addr, tcp_port);

    // Create a message handler
    ragnar_drivers::RagnarJointFeedbackHandler feedback_handler (client.get_joint_names());
    feedback_handler.setNodeHandle(nh);

    // add message handler to manager
    feedback_handler.setCBConnection(client.get_connection());
    client.add_handler(&feedback_handler);

    // run manager (blocking call, does not return)
    client.run();

    ros::waitForShutdown();
  }
  else
  {
    ROS_FATAL(
        "Ragnar feedback interface requires that the 'ip' parameter be set");
  }

  return 0;
}
