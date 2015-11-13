#include "ros_ragnar/ragnar_joint_streamer.h"

// To simplify the massively complicated namespaces
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::simple_message::SimpleMessage;
using industrial::joint_traj_pt_message::JointTrajPtMessage;

typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::JointTrajectoryPoint  ros_JointTrajPt;

namespace TransferStates = industrial_robot_client::joint_trajectory_streamer::TransferStates;

// helper function 
static JointTrajPtMessage create_message(int seq, 
                                         const std::vector<double>& joint_pos, 
                                         double velocity, 
                                         double duration)
{
  industrial::joint_data::JointData pos;
  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  for (size_t i=0; i<joint_pos.size(); ++i)
    pos.setJoint(i, joint_pos[i]);

  rbt_JointTrajPt pt;
  pt.init(seq, pos, velocity, duration);

  JointTrajPtMessage msg;
  msg.init(pt);

  return msg;
}

bool ros_ragnar::RagnarTrajectoryStreamer::transform(const trajectory_msgs::JointTrajectoryPoint& pt_in, trajectory_msgs::JointTrajectoryPoint* pt_out)
{
  ROS_WARN_STREAM("TRANSFORMING TRAJECTORY POINT");
  const static double RAD_TO_DEG = 180.0 / M_PI;
  const static double MS_TO_MMS = 1000.0; 
  *pt_out = pt_in;
  // Radians -> Degrees
  for (size_t i = 0; i < pt_out->positions.size(); ++i)
  {
    pt_out->positions[i] *= RAD_TO_DEG;
  }
  // m/s -> mm/s
  for (size_t i = 0; i < pt_out->velocities.size(); ++i)
  {
    pt_out->velocities[i] *= MS_TO_MMS;
  } 
  for (size_t i = 0; i < pt_out->accelerations.size(); ++i)
  {
    pt_out->accelerations[i] *= MS_TO_MMS;
  }
  return true;
}

//
bool ros_ragnar::RagnarTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, 
                                                  std::vector<JointTrajPtMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

  for (size_t i=0; i<traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // Custom speed calculation for the parallel link robot
    vel = 350.0; // mm/s
    duration = traj->points[i].time_from_start.toSec();

    JointTrajPtMessage msg = ::create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}
