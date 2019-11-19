#include <iostream>
#include <ros/ros.h>
#include "JointStateListener.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "get_joint_angle_node");
	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	JointStateListener joint_state_lintener;
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_lintener);

	while (ros::ok()) {
		ROS_INFO_STREAM(joint_state_lintener.joint_state);
		std::cout << "=================================" << std::endl;
	}

	ros::waitForShutdown();
	return 0;
}
