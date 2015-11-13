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

// 
bool ros_ragnar::RagnarTrajectoryStreamer::init(SmplMsgConnection* connection, 
                                                    const std::vector<std::string> &joint_names,
                                                    const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = industrial_robot_client::joint_trajectory_streamer::TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&RagnarTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return false;
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
    vel = 150.0; // mm/s
    duration = traj->points[i].time_from_start.toSec();

    JointTrajPtMessage msg = ::create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}

// not a virtual override
void ros_ragnar::RagnarTrajectoryStreamer::streamingThread()
{
  JointTrajPtMessage jtpMsg;
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      // this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, reply;
        
    switch (this->state_)
    {
      case TransferStates::IDLE:
        ros::Duration(0.250).sleep();  //  slower loop while waiting for new trajectory
        ROS_WARN("IDLE");
        break;

      case TransferStates::STREAMING:
        ROS_WARN("STREAM STATE");
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
          ROS_INFO("Trajectory streaming complete, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
          break;
        }

        if (!this->connection_->isConnected())
        {
          ROS_WARN("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        jtpMsg = this->current_traj_[this->current_point_];
        jtpMsg.toRequest(msg);
            
        ROS_WARN("Sending joint trajectory point");
        if (this->connection_->sendAndReceiveMsg(msg, reply, false))
        {
          ROS_WARN("Point[%d of %d] sent to controller",
                   this->current_point_, (int)this->current_traj_.size());
          this->current_point_++;
        }
        else
          ROS_WARN("Failed sent joint point, will try again");

        break;
      default:
        ROS_ERROR("Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}
