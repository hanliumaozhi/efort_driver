//
// Created by han on 17-1-2.
//

#include "efort_driver/efort_hardware_interface.h"


unsigned int EfortHardwareInterface::timeout_error = 0;
unsigned int EfortHardwareInterface::cyclic_counter = 0;

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

    next_command_.resize(num_joints_);

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

    ecrt_master_receive(EcatAdmin::master);
    ecrt_domain_process(EcatAdmin::domain1);
    for (std::size_t i = 1; i <= num_joints_; ++i) {
        uint16_t status = EC_READ_U16(EcatAdmin::domain1_pd + 
                            Motor::regs[EcatAdmin::slaves_dict[i]->get_status_word()]) & AKD_STS_MASK;
        if (status == AKD_STS_SWION_DIS && next_command_[(i-1)] != AKD_CMD_DIS_QSTOP) {
	        ROS_ERROR( "AKD: disable quick stop\n" );
            next_command_[(i-1)] = AKD_CMD_DIS_QSTOP;

        }else if (status == AKD_STS_RDY_SWION && next_command_[(i-1)] != AKD_CMD_ENA_SWION ) {
	        ROS_INFO("AKD: enable switch on\n" );
            next_command_[(i-1)] = AKD_CMD_ENA_SWION;
        } else if ( status == AKD_STS_SWION_ENA  && next_command_[(i-1)] != AKD_CMD_ENA_OP ) {
	        ROS_INFO("AKD: start operation\n" );
            next_command_[(i-1)] = AKD_CMD_ENA_OP;
        } else if ( status == AKD_STS_ERROR && next_command_[(i-1)] != AKD_CMD_CLR_ERROR ) {
	        if ( timeout_error ) {
	            if (timeout_error == TIMEOUT_CLEAR_ERROR) {
		        ROS_ERROR( "AKD: ERROR, wait for timeout\n" );
	        }
	        timeout_error--;
	        } else {
	            timeout_error = TIMEOUT_CLEAR_ERROR;
	            next_command_[(i-1)] = AKD_CMD_CLR_ERROR;
	            ROS_INFO( "AKD: clear error now\n" );
    	    }
        }
        int position_in_pluse = EC_READ_S32(EcatAdmin::domain1_pd + Motor::regs[EcatAdmin::slaves_dict[i]->get_t_position()]);
        joint_position_[(i-1)] = EcatAdmin::slaves_dict[i]->pulse_to_radian(position_in_pluse);

		joint_velocity_[(i-1)] = 0;
        joint_effort_[(i-1)] = 0;
	}
    EcatAdmin::check_domain_state();

    ++cyclic_counter;
    if (!(cyclic_counter % 500)) {
        EcatAdmin::check_master_state();
    }

}

void EfortHardwareInterface::write()
{
    //todo
     for (std::size_t i = 1; i <= num_joints_; ++i) {
        int target_position = EcatAdmin::slaves_dict[i]->radian_to_pulse(joint_position_command_[(i-1)]);
        EC_WRITE_S32( EcatAdmin::domain1_pd + Motor::regs[EcatAdmin::slaves_dict[i]->get_r_position()], target_position);
        EC_WRITE_U16( EcatAdmin::domain1_pd + Motor::regs[EcatAdmin::slaves_dict[i]->get_control_word()], next_command_[(i-1)]);
     }

    ecrt_domain_queue(EcatAdmin::domain1);
    ecrt_master_send(EcatAdmin::master);
}
