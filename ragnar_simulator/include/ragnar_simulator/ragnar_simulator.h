#ifndef RAGNAR_SIMULATOR_H
#define RAGNAR_SIMULATOR_H

#include <vector>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>

namespace ragnar_simulator
{

class RagnarSimulator
{
public:
  RagnarSimulator(const std::vector<double>& seed_pose, const std::vector<std::string>& joint_names);

  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  // Initializes trajectory, start time, and position fields
  bool setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory);
  // Computes the robot position at a given time based on the currently active
  // trajectory
  bool computeTrajectoryPosition(const ros::Time& tm, std::vector<double>& output) const;

private:
  std::vector<std::string> joint_names_;
  // State 
  trajectory_msgs::JointTrajectory traj_;
  std::vector<double> traj_start_position_; 
  ros::Time traj_start_time_;
};

}

#endif
