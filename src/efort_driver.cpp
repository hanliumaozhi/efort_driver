#include <time.h>

#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>


#include "efort_driver/efort_hardware_interface.h"



int main(int argc, char **argv) 
{
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
    exit(0);
}