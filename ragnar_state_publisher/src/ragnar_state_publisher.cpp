#include "ragnar_state_publisher/ragnar_state_publisher.h"
#include "ragnar_kinematics/ragnar_kinematics.h"

static tf::Vector3 toTF(const Eigen::Vector3f& v)
{
  return tf::Vector3(v(0), v(1), v(2));
}

static void calculateDirectedTransform(const Eigen::Vector3f& start,
                                       const Eigen::Vector3f& stop,
                                       tf::Transform& transform)
{
  tf::Vector3 origin;
  tf::Quaternion pose;

  origin.setX(start(0));
  origin.setY(start(1));
  origin.setZ(start(2));

  transform.setOrigin(origin);

  tf::Vector3 n = toTF(stop) - toTF(start);
  n.normalize();
  tf::Vector3 z;
  z.setZ(1.0);
  tf::Vector3 y;
  y = n.cross(z);
  y.normalize();
  z = n.cross(y);

  tf::Matrix3x3 rotation(
       n.getX(), y.getX(), z.getX(),
       n.getY(), y.getY(), z.getY(),
       n.getZ(), y.getZ(), z.getZ());

  transform.setBasis(rotation);
}

static
void calculateLinkTransforms(const Eigen::Vector3f& a,
                             const Eigen::Vector3f& b,
                             const Eigen::Vector3f& c,
                             tf::Transform& upper_tf,
                             tf::Transform& lower_tf)
{
  calculateDirectedTransform(a, b, upper_tf);
  calculateDirectedTransform(b, c, lower_tf);
}


namespace rsp = ragnar_state_publisher;

rsp::RagnarStatePublisher::RagnarStatePublisher(const std::string& joints_topic)
{
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>(
                      joints_topic, 1,
                      boost::bind(&rsp::RagnarStatePublisher::updateJointPosition,
                                  this,
                                  _1));
}

void rsp::RagnarStatePublisher::updateJointPosition(const sensor_msgs::JointStateConstPtr& joints)
{
  ROS_INFO("Handling new joint state, calculating tf");
  // using joint states, calculate forward kinematics of ragnar
  float actuators[4] = {joints->position[0], joints->position[1],
                        joints->position[2], joints->position[3]};

  float pose[4];

  if (!ragnar_kinematics::forward_kinematics(actuators, pose))
  {
    ROS_WARN("Could not calculate FK for given pose");
    return;
  }

  // using joints and FK, calculate intermediate points on the robot
  ragnar_kinematics::IntermediatePoints pts;
  ragnar_kinematics::calcIntermediatePoints(actuators, pose, pts);

  // using intermediate points, calculate transform to each link in model

  tf::Transform upper_link, lower_link;
  // Joint 1
  calculateLinkTransforms(pts.A.col(0), pts.B.col(0), pts.C.col(0), upper_link, lower_link);
  tf_broadcaster_.sendTransform(tf::StampedTransform(upper_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "upper_arm_1"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(lower_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "lower_arm_1"));
  // Joint 2
  calculateLinkTransforms(pts.A.col(1), pts.B.col(1), pts.C.col(1), upper_link, lower_link);
  tf_broadcaster_.sendTransform(tf::StampedTransform(upper_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "upper_arm_2"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(lower_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "lower_arm_2"));
  // Joint 3
  calculateLinkTransforms(pts.A.col(2), pts.B.col(2), pts.C.col(2), upper_link, lower_link);
  tf_broadcaster_.sendTransform(tf::StampedTransform(upper_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "upper_arm_3"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(lower_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "lower_arm_3"));
  // Joint 4
  calculateLinkTransforms(pts.A.col(3), pts.B.col(3), pts.C.col(3), upper_link, lower_link);
  tf_broadcaster_.sendTransform(tf::StampedTransform(upper_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "upper_arm_4"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(lower_link,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "lower_arm_4"));
  // world -> base_link
  tf::Transform world_tf = tf::Transform::getIdentity();
  tf_broadcaster_.sendTransform(tf::StampedTransform(world_tf,
                                                     joints->header.stamp,
                                                     "world",
                                                     "base_link"));
}
