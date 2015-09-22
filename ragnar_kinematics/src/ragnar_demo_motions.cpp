#include <ros/ros.h>
#include <ragnar_kinematics/ragnar_kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>

static void populateHeader(std_msgs::Header& header)
{
  header.frame_id = "base_link";
  header.stamp = ros::Time::now();
}

static trajectory_msgs::JointTrajectory makeCircleTrajectory()
{
  using namespace trajectory_msgs;
  // Header
  JointTrajectory traj;
  populateHeader(traj.header);

  // Create circle points
  const double r = 0.1;
  const double dt = 0.05;

  double pose[4];
  double joints[4];

  double total_t = dt;

  for (int i = 0; i < 360; ++i)
  {
    pose[0] = r * std::cos(i * M_PI / 180.0);
    pose[1] = r * std::sin(i * M_PI / 180.0);
    pose[2] = -0.3;
    pose[3] = 0.0;

    JointTrajectoryPoint pt;
    if (!ragnar_kinematics::inverse_kinematics(pose, joints))
    {
      ROS_WARN_STREAM("Could not solve for: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3]);
    }
    else
    {
      pt.positions.assign(joints, joints+4);
      pt.time_from_start = ros::Duration(total_t);
      total_t += dt;
      traj.points.push_back(pt);
    }
  }
  return traj;
}

// Linear trajectory helpers
struct RagnarPoint {
  double joints[4];
};

struct RagnarPose {
  RagnarPose() {}

  RagnarPose(double x, double y, double z)
  {
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = 0.0;
  }

  double pose[4];
};

RagnarPose interpPose(const RagnarPose& start, const RagnarPose& stop, double ratio)
{
  RagnarPose result;
  result.pose[0] = start.pose[0] + ratio * (stop.pose[0] - start.pose[0]);
  result.pose[1] = start.pose[1] + ratio * (stop.pose[1] - start.pose[1]);
  result.pose[2] = start.pose[2] + ratio * (stop.pose[2] - start.pose[2]);
  result.pose[3] = start.pose[3] + ratio * (stop.pose[3] - start.pose[3]);
  return result;
}

bool linearMove(const RagnarPose& start, const RagnarPose& stop, double ds, std::vector<RagnarPoint>& out)
{
  std::vector<RagnarPoint> pts;
  double delta_x = stop.pose[0] - start.pose[0];
  double delta_y = stop.pose[1] - start.pose[1];
  double delta_z = stop.pose[2] - start.pose[2];
  double delta_s = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

  unsigned steps = static_cast<unsigned>(delta_s / ds) + 1;

  for (unsigned i = 0; i <= steps; i++)
  {
    double ratio = static_cast<double>(i) / steps;
    RagnarPose pose = interpPose(start, stop, ratio);
    RagnarPoint pt;
    if (!ragnar_kinematics::inverse_kinematics(pose.pose, pt.joints))
    {
      return false;
    }
    pts.push_back(pt);
  }

  out = pts;
  return true;
}

static trajectory_msgs::JointTrajectory makeLineTrajectory()
{
  using namespace trajectory_msgs;
  // Header
  JointTrajectory traj;
  populateHeader(traj.header);

  std::vector<RagnarPoint> points;
  RagnarPose start (-0.1, -0.1, -0.3);
  RagnarPose stop (0.1, 0.1, -0.4);

  if (!linearMove(start, stop, 0.01, points))
  {
    throw std::runtime_error("Linear movement planning failed");
  }
  
  const double dt = 0.1;
  double total_t = dt;

  for (unsigned i = 0; i < points.size(); ++i)
  {
    JointTrajectoryPoint pt;  
    pt.positions.assign(points[i].joints, points[i].joints+4);
    pt.time_from_start = ros::Duration(total_t);
    total_t += dt;
    traj.points.push_back(pt);
  }
  return traj;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_demo_motions");

  ros::NodeHandle nh;
  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_command", 1);

  // trajectory_msgs::JointTrajectory traj = makeLineTrajectory();
  trajectory_msgs::JointTrajectory traj = makeCircleTrajectory(); 
  ros::Duration(0.5).sleep();

  traj_pub.publish(traj);

  ros::spin();
}
