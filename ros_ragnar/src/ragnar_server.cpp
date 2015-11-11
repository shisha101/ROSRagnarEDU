#include <ros/ros.h>

#include <ros_ragnar/ragnar_message_manager.h>

int DEFAULT_TCP_PORT = 11000;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int tcp_port;
  pnh.param<int>("tcp_port", tcp_port, DEFAULT_TCP_PORT);


  RagnarMessageManager manager;
  manager.init(tcp_port);

  ros::spin();
  ros::waitForShutdown();
  return 0;
}
