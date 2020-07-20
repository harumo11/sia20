#pragma once

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointStateListener {
	public:
		JointStateListener();
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_state;
};

JointStateListener::JointStateListener(){
	// 必ずここで，joint_stateの初期化を行ってください
	// まだメッセージが届いていないときに読み出そうとするとセグフォを起こします．
	// 文字列および，ROS_INFOだけならからの文字が渡されるだけなので，セグフォは起こりません．

}

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_state = msgs;
}

