#ifndef RAGNAR_STATE_PUBLISHER_H
#define RAGNAR_STATE_PUBLISHER_H

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace ragnar_state_publisher
{

class RagnarStatePublisher
{
public:
  RagnarStatePublisher(const std::string& joints_topic, const std::string& prefix = "");

  void updateJointPosition(const sensor_msgs::JointStateConstPtr& joints);

private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber joint_sub_;

  tf::Transform base_transform_;
  std::vector<tf::Vector3> zi_;
  std::string prefix_;
};

}

#endif
