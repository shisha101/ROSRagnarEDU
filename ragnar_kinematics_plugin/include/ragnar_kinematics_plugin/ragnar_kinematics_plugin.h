#ifndef RAGNAR_KINEMATICS_PLUGIN_H
#define RAGNAR_KINEMATICS_PLUGIN_H
/* Author: Shehabeldin Abdelgawad */

#include <moveit/kinematics_base/kinematics_base.h>
#include <ragnar_kinematics/ragnar_kinematics.h>
#include <string>
// messages
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometry_msgs/Pose.h>

//namespace ragnar_kinematics_plugin {

class ragnar_kinematics_plugin : public kinematics::KinematicsBase{
public:
    /*!
     * @brief ragnar_kinematics_plugin, constructor for the class
     * @param x dummy variable
     */
    ragnar_kinematics_plugin(); // @TODO: implement

    ~ragnar_kinematics_plugin(){}

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    const std::vector<double> &consistency_limits,
                                    std::vector<double> &solution,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    std::vector<double> &solution,
                                    const IKCallbackFn &solution_callback,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    const std::vector<double> &consistency_limits,
                                    std::vector<double> &solution,
                                    const IKCallbackFn &solution_callback,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                 const std::vector<double> &joint_angles,
                                 std::vector<geometry_msgs::Pose> &poses) const;

    virtual bool initialize(const std::string& robot_description,
                            const std::string& group_name,
                            const std::string& base_frame,
                            const std::string& tip_frame,
                            double search_discretization);

    virtual const std::vector<std::string>& getLinkNames() const;

    virtual const std::vector<std::string>& getJointNames() const;

private:



};

#endif
//}
