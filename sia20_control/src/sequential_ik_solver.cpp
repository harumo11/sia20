#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "quaternion_utility.hpp"
#include "target_velocity.hpp"


int main(int argc, char* argv[])
{

	// ROS Initialize
	ros::init(argc, argv, "sequential_ik_solver");
	ros::NodeHandle node_handle;

	// AsyncSpinner Initialize 
	//ros::AsyncSpinner spinner(1);
	//spinner.start();
	//ROS_INFO_STREAM("Is spinner start : " << std::boolalpha << spinner.canStart());

	// Configuration
	const std::string PLANNING_GROUP = "sia20_arm";
	const unsigned int PUBLISH_CYCLE = 50;	// 50[Hz]

	// Make velocity subscriber
	TargetVelocity target_velocity;
	ros::Subscriber target_velocity_subscriber = node_handle.subscribe("target_velocity", 10, &TargetVelocity::call_back, &target_velocity);

	// Timer setting
	ros::Rate timer(10);

	while (ros::ok()) {
		ROS_WARN_STREAM("SPIN ONCE");
		std::cout << "spin once" << std::endl;
		ros::spinOnce();
		timer.sleep();
	}
	
	return 0;
}
