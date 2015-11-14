#include "ros_ragnar/ragnar_joint_streamer.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_joint_streamer_node");
  ros::NodeHandle pnh("~");

  // Load robot parameters
  std::string robot_ip;
  pnh.param<std::string>("robot_ip", robot_ip, "");

  int port;
  pnh.param<int>("port", port,
                 (int)industrial::simple_socket::StandardSocketPorts::MOTION);

  // Create streamer and run
  ros_ragnar::RagnarTrajectoryStreamer streamer;
  streamer.init(robot_ip, port);
  streamer.run();
  return 0;
}
