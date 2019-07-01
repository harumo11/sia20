#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>


class TargetVelocity : public geometry_msgs::Twist
{
public:
	TargetVelocity();
	void call_back(const geometry_msgs::TwistConstPtr target_velocity_data);
	void print_data();
	geometry_msgs::Twist get();
};

TargetVelocity::TargetVelocity(){

	// clear linear
	this->linear.x = 0;
	this->linear.y = 0;
	this->linear.z = 0;

	// clear angular
	this->angular.x = 0;
	this->angular.y = 0;
	this->angular.z = 0;
}

// callback is called when target velocity command is arrived.
void TargetVelocity::call_back(const geometry_msgs::TwistConstPtr target_velocity_data){

	// Move listened data to memeber 
	this->linear = target_velocity_data->linear;
	this->angular = target_velocity_data->angular;
	this->print_data();
}

// get target velocity data
geometry_msgs::Twist TargetVelocity::get(){

	geometry_msgs::Twist twist;
	twist.linear = this->linear;
	twist.angular = this->angular;

	return twist;
}

// print target velocity data
void TargetVelocity::print_data(){
	ROS_DEBUG_STREAM_THROTTLE(10, "Target velocity : " << "x :" << this->linear.x  << "\t"
	                                     << "y : " << this->linear.y  << "\t"
	                                     << "z : " << this->linear.z  << "\t"
	                                     << "r : " << this->angular.x << "\t"
	                                     << "p : " << this->angular.y << "\t"
	                                     << "w : " << this->angular.z << "\t"
										 );
}
