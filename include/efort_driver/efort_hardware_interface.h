//
// Created by han on 17-1-2.
//

#ifndef EFORT_DRIVER_EFORT_HARDWARE_INTERFACE_H
#define EFORT_DRIVER_EFORT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//for ethercat
#include "EcatAdmin.h"

class EfortHardwareInterface : public hardware_interface::RobotHW {
public:
    explicit EfortHardwareInterface(ros::NodeHandle& nh);

    virtual void init();

    virtual void read();

    virtual void write();

protected:

    ros::NodeHandle nh_;


    //publish joint status
    hardware_interface::JointStateInterface joint_state_interface_;

    //get command from JointTrajectoryController
    hardware_interface::PositionJointInterface position_joint_interface_;

    std::vector<std::string> joint_names_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    //not effort
    std::vector<double> joint_effort_;

    std::vector<double> joint_position_command_;

    std::vector<uint16_t> next_command_;

    std::size_t num_joints_;

};


#endif //EFORT_DRIVER_EFORT_HARDWARE_INTERFACE_H
