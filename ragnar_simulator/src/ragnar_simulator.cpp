#include "ragnar_simulator/ragnar_simulator.h"
#include <ros/console.h>

ragnar_simulator::RagnarSimulator::RagnarSimulator(const std::vector<double>& seed_pose)
  : traj_start_position_(seed_pose)
  , traj_start_time_(ros::Time::now())
{
  if (traj_start_position_.size() != 4)
    throw std::runtime_error("Size of Ragnar simulator seed pose != 4");

  joint_names_.push_back("ragnar_joint1");
  joint_names_.push_back("ragnar_joint2");
  joint_names_.push_back("ragnar_joint3");
  joint_names_.push_back("ragnar_joint4");
}

bool ragnar_simulator::RagnarSimulator::setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory)
{
  ROS_INFO("Setting new active trajectory");
  // Compute current state
  ros::Time now = ros::Time::now();
  std::vector<double> position;
  computeTrajectoryPosition(now, position);

  // Rollover to the new trajectory
  traj_start_position_ = position;
  traj_ = new_trajectory;
  traj_start_time_ = now;

  return true;
}

static double linearInterpolate(double start, double stop, double ratio)
{
  return start + (stop - start) * ratio;
}

  // Computes the robot position at a given time based on the currently active
  // trajectory
bool ragnar_simulator::RagnarSimulator::computeTrajectoryPosition(const ros::Time& tm, 
                                                                  std::vector<double>& output) const
{
  // Check to see if time is in past of traj
  if (tm < traj_start_time_ || traj_.points.empty())
  {
    output = traj_start_position_;
    return true;
  }
  // check to see if time is past end of traj
  else if (tm > traj_start_time_ + traj_.points.back().time_from_start)
  {
    output = traj_.points.back().positions;
    return true;
  }
  
  // Otherwise the traj must be within the trajectory
  ros::Duration dt = tm - traj_start_time_;

  size_t idx = 0;
  for (size_t i = 0; i < traj_.points.size(); ++i)
  {
    if (dt < traj_.points[i].time_from_start)
    {
      idx = i;
      break;
    }
  }

  // Grab the two points and interpolate
  const trajectory_msgs::JointTrajectoryPoint& end_pt = traj_.points[idx];

  // output container
  std::vector<double> point;
  point.reserve(traj_start_position_.size());

  if (idx == 0)
  {
    // interpolate from start position
    double ratio = dt.toSec() / end_pt.time_from_start.toSec();

    for (int i = 0; i < 4; ++i)
    {
      point.push_back(linearInterpolate(traj_start_position_[i], end_pt.positions[i], ratio));
    }
  }
  else
  {
    const trajectory_msgs::JointTrajectoryPoint& start_pt = traj_.points[idx-1];
    // interpolate between two points
    double ratio = (dt - start_pt.time_from_start).toSec() / (end_pt.time_from_start - start_pt.time_from_start).toSec();   

    for (int i = 0; i < 4; ++i)
    {
      point.push_back(linearInterpolate(start_pt.positions[i], end_pt.positions[i], ratio));
    }
  }

  output = point;
  return true;
}
