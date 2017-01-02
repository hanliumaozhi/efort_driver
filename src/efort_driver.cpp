#include <time.h>

#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>


#include "efort_driver/efort_hardware_interface.h"



int main(int argc, char **argv) 
{
    EcatAdmin::add_motor(1, "joint_1", 1, 0, 0x000002E1, 0x00000000);
    //
    //EcatAdmin::slaves_dict[1]->setting_trans(double offset, double param_1, double param_2, double param_3);
    EcatAdmin::add_motor(2, "joint_2", 2, 0, 0x000002E1, 0x00000000);
    //EcatAdmin::slaves_dict[2]->setting_trans(double offset, double param_1, double param_2, double param_3);
    EcatAdmin::add_motor(3, "joint_3", 3, 0, 0x000002E1, 0x00000000);
    //EcatAdmin::slaves_dict[3]->setting_trans(double offset, double param_1, double param_2, double param_3);
    EcatAdmin::add_motor(4, "joint_4", 4, 0, 0x000002E1, 0x00000000);
    //EcatAdmin::slaves_dict[4]->setting_trans(double offset, double param_1, double param_2, double param_3);
    EcatAdmin::add_motor(5, "joint_5", 5, 0, 0x000002E1, 0x00000000);
    //EcatAdmin::slaves_dict[5]->setting_trans(double offset, double param_1, double param_2, double param_3);
    EcatAdmin::add_motor(6, "joint_6", 6, 0, 0x000002E1, 0x00000000);
    //EcatAdmin::slaves_dict[6]->setting_trans(double offset, double param_1, double param_2, double param_3);


    EcatAdmin::start_for_ros_control();
    ros::init(argc, argv, "efort_driver");
	ros::NodeHandle nh;

    std::shared_ptr<EfortHardwareInterface> hardware_interface_ = std::make_shared<EfortHardwareInterface>(nh);
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_ = std::make_shared<controller_manager::ControllerManager>(hardware_interface_.get(), nh);

    ros::AsyncSpinner spinner(3);
	spinner.start();

    ros::Duration elapsed_time;
	struct timespec last_time, current_time;
	static const double BILLION = 1000000000.0;
    
    clock_gettime(CLOCK_MONOTONIC, &last_time);
    while(ros::ok()){
        hardware_interface_->read();

		clock_gettime(CLOCK_MONOTONIC, &current_time);
		elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec)/ BILLION);
		ros::Time ros_time = ros::Time::now();
		controller_manager_->update(ros_time, elapsed_time);
		last_time = current_time;

		hardware_interface_->write();
    }

    spinner.stop();
    EcatAdmin::shutdown();
    exit(0);
}