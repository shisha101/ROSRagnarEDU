#include "ragnar_kinematics_plugin/ragnar_kinematics_plugin.h"

ragnar_kinematics_plugin::ragnar_kinematics_plugin()
{

}

bool ragnar_kinematics_plugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    double cartesian_goal_mm[4]; // x, y, z, angle ?
    double result[4];
    bool IK_result;

    // data conversion
    cartesian_goal_mm[0] = ik_pose.position.x;
    cartesian_goal_mm[1] = ik_pose.position.y;
    cartesian_goal_mm[2] = ik_pose.position.z;
    cartesian_goal_mm[3] = 0.0; // temp value
    IK_result = ragnar_kinematics::inverse_kinematics(cartesian_goal_mm, result);
    return IK_result;
}

bool ragnar_kinematics_plugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              std::vector<double> &solution,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options) const
{
    return true;
}

bool ragnar_kinematics_plugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options) const
{
    return true;
}

bool ragnar_kinematics_plugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options) const
{
    return true;
}

bool ragnar_kinematics_plugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options) const
{
    return true;
}

bool ragnar_kinematics_plugin::getPositionFK(const std::vector<std::string> &link_names,
                                                     const std::vector<double> &joint_angles,
                                                     std::vector<geometry_msgs::Pose> &poses) const
{
    return true;
}

bool ragnar_kinematics_plugin::initialize(const std::string& robot_description,
                        const std::string& group_name,
                        const std::string& base_frame,
                        const std::string& tip_frame,
                        double search_discretization)
{
    return true;
}

const std::vector<std::string>& ragnar_kinematics_plugin::getLinkNames() const
{
    const std::vector<std::string> s;
    return s;
}

const std::vector<std::string>& ragnar_kinematics_plugin::getJointNames() const
{
    const std::vector<std::string> s;
    return s;
}

