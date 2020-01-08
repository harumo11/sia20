// このプログラムはpose_follow_plannerから受け取った命令値をsia20に
// 送るためのプログラムです．

#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_state;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_state = msgs;
}

class JointTargetListener {
	public:
		void call_back(const trajectory_msgs::JointTrajectoryPoint msgs);
		trajectory_msgs::JointTrajectoryPoint joint_target;
};

void JointTargetListener::call_back(const trajectory_msgs::JointTrajectoryPoint msgs){
	this->joint_target = msgs;
}

void set_joint_trajectry_config(trajectory_msgs::JointTrajectory& joint_trajectory_msgs){
	//set joint name
	joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
	//set stamp
	joint_trajectory_msgs.header.stamp = ros::Time::now();
}

void set_joint_trajectry_point_config(trajectory_msgs::JointTrajectoryPoint& joint_trajectory_point, const double time_from_start){
	joint_trajectory_point.time_from_start = ros::Duration(time_from_start); 
	joint_trajectory_point.velocities = {0, 0, 0, 0, 0, 0, 0};
}

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "joint_trajectory_publisher");

	// create ros node handle
	ros::NodeHandle node_handle;

	// make publisher. NOTE use same topic name between Publisher and Subscriber.
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/joint_command", 1);

	// make subscriber. 
	JointStateListener joint_state_listener;
	JointTargetListener joint_target_listener;
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
	ros::Subscriber target_subscriber = node_handle.subscribe("/joint_value_command", 1, &JointTargetListener::call_back, &joint_target_listener);

	// make loop timer
	//ros::Rate rate_timer(10);
	const double control_cycle = 10;
	ros::Rate rate_timer(control_cycle);

	int counter = 0;
	//const double T = 0.1;
	const double T = 1.0 / control_cycle;
	std::cout << "T : " << T << std::endl;
	const double step_size = 0.01;

	ROS_INFO_STREAM("Waiting for /joint_value_command topic");
	ros::topic::waitForMessage<trajectory_msgs::JointTrajectoryPoint>("/joint_value_command");

	// subscribe current position (first time)
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	set_joint_trajectry_config(joint_trajectory_msgs);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
	set_joint_trajectry_point_config(joint_trajectory_point, counter++ * T);
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	joint_trajectory_point.positions = joint_state_listener.joint_state.position;

	// publish current position (first time)
	joint_trajectory_msgs.points.push_back(joint_trajectory_point);
	joint_trajectory_publisher.publish(joint_trajectory_msgs);
	
	while (ros::ok()) {
		//ROS_INFO_STREAM(joint_target_listener.joint_target);
		//ROS_INFO_STREAM(joint_state_listener.joint_state);

		// make messages for publishing
		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
		set_joint_trajectry_config(joint_trajectory_msgs);
		trajectory_msgs::JointTrajectoryPoint joint_trajectory_point = joint_target_listener.joint_target;
		set_joint_trajectry_point_config(joint_trajectory_point, counter++ * T);
		//joint_trajectory_point.positions = joint_state_listener.joint_state.position;

		//// revolute joint s angle
		//joint_trajectory_point.positions.at(0) += step_size;

		ROS_INFO_STREAM(joint_trajectory_point);

		joint_trajectory_msgs.points.push_back(joint_trajectory_point);
		joint_trajectory_publisher.publish(joint_trajectory_msgs);

		ROS_INFO("Publish once");

		ros::spinOnce();
		rate_timer.sleep();
	}

	return 0;
}
