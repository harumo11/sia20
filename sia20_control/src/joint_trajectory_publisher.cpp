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
	ROS_INFO_STREAM(this->joint_state);
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
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);

	// make loop timer
	ros::Rate rate_timer(10);

	int counter = 0;
	double step_size = 0.01;

	ros::Duration(2).sleep();
	ros::spinOnce();

	// subscribe current position (first time)
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	set_joint_trajectry_config(joint_trajectory_msgs);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
	set_joint_trajectry_point_config(joint_trajectory_point, counter++ * 0.1);
	joint_trajectory_point.positions = joint_state_listener.joint_state.position;

	// publish current position (first time)
	joint_trajectory_msgs.points.push_back(joint_trajectory_point);
	joint_trajectory_publisher.publish(joint_trajectory_msgs);
	
	while (ros::ok()) {

		// make messages for publishing
		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
		set_joint_trajectry_config(joint_trajectory_msgs);
		trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
		set_joint_trajectry_point_config(joint_trajectory_point, counter++ * 0.1);
		joint_trajectory_point.positions = joint_state_listener.joint_state.position;

		// revolute joint s angle
		joint_trajectory_point.positions.at(0) += step_size;

		std::cout << "size of joint state listener : " << joint_state_listener.joint_state.position.size() << std::endl;
		std::cout << "size of joint trajectory : " << joint_trajectory_point.positions.size() << std::endl;
		std::cout << "time_from_start : " << counter * 0.1 << std::endl;

		joint_trajectory_msgs.points.push_back(joint_trajectory_point);
		joint_trajectory_publisher.publish(joint_trajectory_msgs);

		ROS_INFO("Publish once");

		ros::spinOnce();
		rate_timer.sleep();
	}

	return 0;
}
