#ifndef RAGNAR_KINEMATIC_DEFS_H
#define RAGNAR_KINEMATIC_DEFS_H

/**
 * NOTE THAT THESE PARAMETERS USE METERS AS THEIR
 * PRIMARY UNIT OF LENGTH. This is a departure from
 * the smoothieboard that uses mm and degrees as its
 * primary units of measure.
 */

// a : Base X
#define RAGNAR_JOINT1_BASE_X  0.28f      // param 0
#define RAGNAR_JOINT2_BASE_X  -0.28f     // param 1
#define RAGNAR_JOINT3_BASE_X  -0.28f     // param 2
#define RAGNAR_JOINT4_BASE_X  0.28f      // param 3

// b : Base Y
#define RAGNAR_JOINT1_BASE_Y  0.114f      // param 4
#define RAGNAR_JOINT2_BASE_Y  0.114f      // param 5
#define RAGNAR_JOINT3_BASE_Y  -0.114f     // param 6
#define RAGNAR_JOINT4_BASE_Y  -0.114f     // param 7

// rho : base_pan
#define RAGNAR_JOINT1_BASE_PAN  -0.261799388f     // param 8
#define RAGNAR_JOINT2_BASE_PAN  0.261799388f      // param 9
#define RAGNAR_JOINT3_BASE_PAN  -0.261799388f     // param 10
#define RAGNAR_JOINT4_BASE_PAN  0.261799388f      // param 11

// alpha : base_tilt
#define RAGNAR_JOINT1_BASE_TILT  -0.785398163397448f      // param 12
#define RAGNAR_JOINT2_BASE_TILT  0.785398163397448f      // param 13
#define RAGNAR_JOINT3_BASE_TILT  0.785398163397448f      // param 14
#define RAGNAR_JOINT4_BASE_TILT  -0.785398163397448f      // param 15

// l1 : primary_arm
#define RAGNAR_JOINT1_PRIMARY_ARM  0.300f      // param 16
#define RAGNAR_JOINT2_PRIMARY_ARM  0.300f      // param 17
#define RAGNAR_JOINT3_PRIMARY_ARM  0.300f      // param 18
#define RAGNAR_JOINT4_PRIMARY_ARM  0.300f      // param 19

// l2 : secondary_arm
#define RAGNAR_JOINT1_SECONDARY_ARM  0.550f     // param 20
#define RAGNAR_JOINT2_SECONDARY_ARM  0.550f     // param 21
#define RAGNAR_JOINT3_SECONDARY_ARM  0.550f     // param 22
#define RAGNAR_JOINT4_SECONDARY_ARM  0.550f     // param 23

// c : bridged
#define RAGNAR_JOINT1_BRIDGED_DISTANCE  0.100f  // param 24
#define RAGNAR_JOINT2_BRIDGED_DISTANCE  0.100f  // param 25
#define RAGNAR_JOINT3_BRIDGED_DISTANCE  0.100f  // param 26
#define RAGNAR_JOINT4_BRIDGED_DISTANCE  0.100f  // param 27

// PLATFORM specific parameters (fixed configuration vs X configuration vs hash configuration)
//// Fixed Platform
#define RAGNAR_FIXED_PLATFORM_X_THETA_MIN   0.0f
#define RAGNAR_FIXED_PLATFORM_X_THETA_MAX   0.0f

#define RAGNAR_JOINT1_FIXED_BRIDGE_ANGLE  1.04719755F   // param 28
#define RAGNAR_JOINT2_FIXED_BRIDGE_ANGLE  2.61799388F   // param 29
#define RAGNAR_JOINT3_FIXED_BRIDGE_ANGLE  3.66519143F   // param 30
#define RAGNAR_JOINT4_FIXED_BRIDGE_ANGLE  5.23598776F   // param 31

//// X Platform
#define RAGNAR_X_PLATFORM_X_THETA_MIN   -30.0f
#define RAGNAR_X_PLATFORM_X_THETA_MAX   30.0f

#define RAGNAR_JOINT1_X_BRIDGE_ANGLE  0.523598776F   // param 28
#define RAGNAR_JOINT2_X_BRIDGE_ANGLE  2.617993878F   // param 29
#define RAGNAR_JOINT3_X_BRIDGE_ANGLE  3.665191429F   // param 30
#define RAGNAR_JOINT4_X_BRIDGE_ANGLE  5.759586532F   // param 31

//// Hash Platform
#define RAGNAR_HASH_PLATFORM_X_THETA_MIN   -90.0f
#define RAGNAR_HASH_PLATFORM_X_THETA_MAX   90.0f

#define RAGNAR_JOINT1_HASH_BRIDGE_ANGLE  0.523598776F   // param 28
#define RAGNAR_JOINT2_HASH_BRIDGE_ANGLE  2.617993878F   // param 29
#define RAGNAR_JOINT3_HASH_BRIDGE_ANGLE  3.665191429F   // param 30
#define RAGNAR_JOINT4_HASH_BRIDGE_ANGLE  5.759586532F   // param 31

namespace ragnar_kinematics
{
  extern const float ragnar_params[32];
  extern const float ragnar_cosine_cache[32];
  extern const float ragnar_sine_cache[32];
}

void debugParams();

#endif // RAGNAR_KINEMATIC_DEFS_H

