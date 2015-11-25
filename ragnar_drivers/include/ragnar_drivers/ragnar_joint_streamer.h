#ifndef RAGNAR_JOINT_TRAJECTORY_STREAMER
#define RAGNAR_JOINT_TRAJECTORY_STREAMER

#include "industrial_robot_client/joint_trajectory_streamer.h"

namespace ragnar_drivers
{

class RagnarTrajectoryStreamer
    : public industrial_robot_client::joint_trajectory_streamer::
          JointTrajectoryStreamer
{
public:
  using JointTrajectoryInterface::init;

  virtual bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in,
                         trajectory_msgs::JointTrajectoryPoint* pt_out);

  virtual bool trajectory_to_msgs(
      const trajectory_msgs::JointTrajectoryConstPtr& traj,
      std::vector<industrial::joint_traj_pt_message::JointTrajPtMessage>* msgs);
};
}

#endif
