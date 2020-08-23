#include <iostream>
#include <ros/ros.h>
#include "JointStateListener.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "get_joint_angle_node");
	ros::NodeHandle node_handle;

	ros::Rate timer(40);

	JointStateListener joint_state_lintener;
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_lintener);

	for (int i = 0; i < 100; i++) {
		timer.sleep();
		ros::spinOnce();
	}

	while (ros::ok()) {
		ros::spinOnce();
		timer.sleep();
		ROS_INFO_STREAM(joint_state_lintener.joint_state);
		double a = joint_state_lintener.joint_state.position.at(6);
		std::cout << a << std::endl;
		std::cout << "=================================" << std::endl;
	}

	ros::waitForShutdown();
	return 0;
}
