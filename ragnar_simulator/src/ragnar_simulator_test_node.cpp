#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_sim_test");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);


  static double test_pt[] = {0.897913, -0.667189, -1.266242, -0.123857};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.assign(test_pt, test_pt + 4);
  point.time_from_start = ros::Duration(10.0);

  static double test_pt2[] = {-1.217134, 1.558858, 1.113690, -0.960308};
  trajectory_msgs::JointTrajectoryPoint point2;
  point2.positions.assign(test_pt2, test_pt2 + 4);
  point2.time_from_start = ros::Duration(20.0);


  ros::spinOnce();

  ros::Duration(1.0).sleep();

  trajectory_msgs::JointTrajectory traj;
  traj.header.frame_id = "world";
  traj.header.stamp = ros::Time::now();
  traj.points.push_back(point);
  traj.points.push_back(point2);

  pub.publish(traj);

  ROS_INFO("Published... spinning til closed");
  ros::spin();
}