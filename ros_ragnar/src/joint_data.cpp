/*
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <simple_message/log_wrapper.h>
#include <ros_ragnar/joint_data.h>


namespace joint_data {


        JointData::JointData() {
            this->init();
        }
        JointData::~JointData() {}

        void JointData::init() {
            for(int ii = 0; ii < this->getJointArraySize(); ++ii) {
                this->setJointValue(ii, 0.0);
            }
        }

        bool JointData::setJointValue(industrial::shared_types::shared_int index,
                industrial::shared_types::shared_real value) {
            bool success = false;
            if((index >= 0) && (index < this->getJointArraySize())) {
                this->Joint_values_[index] = value;
                success = true;
            } else {
                LOG_ERROR("Joint index must be between 0 and %d (%d passed in)", this->getJointArraySize() , index);
                success = false;
            }
            return success;
        }

        bool JointData::getJointValue(industrial::shared_types::shared_int index,
                industrial::shared_types::shared_real& value) const {
            bool success = false;
            if((index >= 0) && (index < this->getJointArraySize())) {
                value = this->Joint_values_[index];
                success = true;
            } else {
                LOG_ERROR("Joint index must be between 0 and %d (%d passed in)", this->getJointArraySize() , index);
                success = false;
            }
            return success;
        }

        industrial::shared_types::shared_real JointData::getJointValue(industrial::shared_types::shared_int index) {
            return this->Joint_values_[index];
        }

        bool JointData::operator==(JointData& rhs) {
            bool success = true;
            for(int ii = 0; ii < this->getJointArraySize(); ++ii) {
                if(this->getJointValue(ii) != rhs.getJointValue(ii)) {
                    success = false;
                    break;
                }
            }
            return success;
        }

        void JointData::copyFrom(const JointData& src) {
            for(int ii = 0; ii < this->getJointArraySize(); ++ii) {
                this->setJointValue(ii, this->getJointValue(ii));
            }
        }

        bool JointData::load(industrial::byte_array::ByteArray *buffer) {
            bool success = true;
            industrial::shared_types::shared_real value = 0.0;
            for(int ii = 0; ii < this->getJointArraySize(); ++ii) {
                if(!this->getJointValue(ii, value)) {
                    LOG_ERROR("Error getting joint value %d", ii);
                    success = false;
                    break;
                } else if(!buffer->load(value)) {
                    LOG_ERROR("Error loading joint value %d into buffer", ii);
                    success = false;
                    break;
                }
            }
            return success;
        }


        bool JointData::unload(industrial::byte_array::ByteArray *buffer) {
            bool success = true;
            industrial::shared_types::shared_real value = 0.0;
            for(int ii = this->getJointArraySize() -1; ii >= 0; ++ii) {
                if(!buffer->unload(value)) {
                    LOG_ERROR("Error unloading Joint value %d" , ii);
                    success = false;
                    break;
                } else if(!this->setJointValue(ii, value)) {
                    LOG_ERROR("Error setting Joint value %d", ii);
                    success = false;
                    break;
                }
            }
            return success;            
        }


}; // joint_data

