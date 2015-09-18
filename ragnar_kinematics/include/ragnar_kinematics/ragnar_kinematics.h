#ifndef RAGNAR_KINEMATICS_H
#define RAGNAR_KINEMATICS_H

#include <eigen3/Eigen/Dense>

namespace ragnar_kinematics
{

bool inverse_kinematics(const float cartesian_mm[], float actuator_mm[]);
bool forward_kinematics(const float actuator_mm[], float cartesian_mm[]);

struct IntermediatePoints
{
  const static int N_ARMS = 4;
  const static int N_DIMS = 3;
  typedef Eigen::Matrix<float, N_DIMS, N_ARMS> ArmMatrixf;

  ArmMatrixf A; // position of motors
  ArmMatrixf B; // position of elbow
  ArmMatrixf C; // position of mount to eff plate

  // Alignment support
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool calcIntermediatePoints(const float actuator_mm[], const float cartesian_mm[],
                            IntermediatePoints& pts);

}

#endif // RAGNAR_KINEMATICS_H

