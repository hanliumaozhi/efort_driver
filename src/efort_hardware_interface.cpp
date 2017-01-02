//
// Created by han on 17-1-2.
//

#include "efort_driver/efort_hardware_interface.h"


EfortHardwareInterface::EfortHardwareInterface(ros::NodeHandle& nh)
{
    nh_ = nh;
    init();
}

void EfortHardwareInterface::init()
{
    nh_.getParam("hardware_interface/joints", joint_names_);

    num_joints_ = joint_names_.size();

    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);

    joint_position_command_.resize(num_joints_);

    for(std::size_t i = 0; i != num_joints_; ++i){
        joint_state_interface_.registerHandle(
                hardware_interface::JointStateHandle(joint_names_[i],
                                                     &joint_position_[i], &joint_velocity_[i],
                                                     &joint_effort_[i]));

        position_joint_interface_.registerHandle(
                hardware_interface::JointHandle(
                        joint_state_interface_.getHandle(joint_names_[i]),
                        &joint_position_command_[i]));
    }
    registerInterface(&joint_state_interface_); 
	registerInterface(&position_joint_interface_);

}

void EfortHardwareInterface::read()
{
    //todo
}

void EfortHardwareInterface::write()
{
    //todo
}
