#include "ros_ragnar/ragnar_joint_streamer.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_joint_streamer_node");

  const std::string ip = "192.168.1.240";

  ros_ragnar::RagnarTrajectoryStreamer streamer;

  streamer.init(ip);
  streamer.run();
  return 0;
}