#ifndef JOINT_DATA_H
#define JOINT_DATA_H
/*
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <simple_message/simple_message.h>       // Simple message type
#include <simple_message/simple_serialize.h>     // Serialization code
#include <simple_message/shared_types.h>         // Primitive data types used in simple message


namespace joint_data
{

/**
 * @brief Class representing Joint data for the robot pick position
 *
 * SimpleMessage style class to use to represent the robot pick position.  The Joint is defined as a translation and a
 * ZYX Euler rotation. The position is packed in the following order:
 *
 *  -# The X translation, in meters
 *  -# The Y translation, in meters
 *  -# The Z translation, in meters
 *  -# The Z Euler angle, in degrees
 *  -# The Y Euler angle, in degrees
 *  -# The X Euler angle, in degrees
 */
class JointData: public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * @brief Creates a default JointData; calls JointData::init within the constructor.
   */
  JointData();

  /**
   * @brief Destructor; currently a no-op
   */
  ~JointData();

  /**
   * @brief Initializes (resets) the data structure
   *
   * Sets the translation for the Joint dat to the origin, and clears out any rotations
   */
  void init();

  /**
   * @brief setJointValue
   * @param index
   * @param value
   * @return
   */
  bool setJointValue(industrial::shared_types::shared_int index, industrial::shared_types::shared_real value);

  /**
   * @brief getJointValue
   * @param index
   * @param value
   * @return
   */
  bool getJointValue(industrial::shared_types::shared_int index,
                    industrial::shared_types::shared_real& value) const;
  industrial::shared_types::shared_real getJointValue(industrial::shared_types::shared_int index);

  bool operator==(JointData& rhs);
  void copyFrom(const JointData& src);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return Joint_ARRAY_SIZE * sizeof(industrial::shared_types::shared_real);
  }

  int getJointArraySize() const { return Joint_ARRAY_SIZE; }

private:
  static const industrial::shared_types::shared_int Joint_ARRAY_SIZE = 4;
  industrial::shared_types::shared_real Joint_values_[Joint_ARRAY_SIZE];

}; // JointData

}; //joint_data



#endif//Joint_DATA_H
