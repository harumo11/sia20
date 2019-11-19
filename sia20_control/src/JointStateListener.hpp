#pragma once

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_state;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_state = msgs;
}
