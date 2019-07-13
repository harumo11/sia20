#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_states;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_states = msgs;
	ROS_INFO_STREAM("Joint State : " << msgs.position[0] << " " << msgs.position[1] << " " << msgs.position[2] << " " << msgs.position[3] << " " << msgs.position[4] << " " << msgs.position[5] << " " << msgs.position[6]);
}


int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "joint_trajectory_publisher");

	// create ros node handle
	ros::NodeHandle node_handle;

	// make publisher. NOTE use same topic name between Publisher and Subscriber.
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
	ROS_INFO_STREAM("JointTrajectoryPublisher starts");

	// Async spinner starts
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO_STREAM("AsyncSpinner starts");
	
	// Prepare Loop timer
	ros::Rate short_timer(1);
	ros::Rate timer(0.1);

	// Joint state listener starts
	JointStateListener joint_state_listener;
	ros::Subscriber current_point_listrener = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
	ROS_INFO_STREAM("JointStateListener starts");
	for (int i = 0; i < 5; i++) {
		ros::spinOnce();
		ROS_INFO_STREAM("Initial spin");
		short_timer.sleep();
	}


	while (ros::ok()) {
		
		// make messages for publishing
		ROS_INFO_STREAM("Main loop");
		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
		joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", 
											 "joint_u", "joint_r", "joint_b", "joint_t"};

		// Prepare published message;
		// Currnet position
		trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
		for (int i = 0; i < (int)joint_state_listener.joint_states.position.size(); i++) {
			joint_trajectory_point.positions.push_back(joint_state_listener.joint_states.position.at(i));
			ROS_INFO_STREAM("Current Position : " << joint_trajectory_point.positions.at(i));
		}
		joint_trajectory_point.time_from_start = ros::Duration(0);
		ROS_INFO_STREAM("Prepared publishing message");

		// Add velocity to JointTrajectoryPoint
		//joint_trajectory_point.velocities = {0, 0, 0, 0, 0, 0, 0.10472};
		joint_trajectory_point.velocities = {0, 0, 0, 0, 0, 0, 0};
		ROS_INFO_STREAM("Set velocity in publising message");

		// added link_t angle value
		ROS_INFO_STREAM("JointTrajectoryPoint position size : " << joint_trajectory_point.positions.size());
		ROS_INFO_STREAM("Current : " << joint_trajectory_point.positions[0] 
				                     << " " 
								 	 << joint_trajectory_point.positions[1]
								 	 << " " 
								 	 << joint_trajectory_point.positions[2]
								 	 << " " 
								 	 << joint_trajectory_point.positions[3]
								 	 << " " 
								 	 << joint_trajectory_point.positions[4]
								 	 << " " 
								 	 << joint_trajectory_point.positions[5]
								 	 << " " 
								 	 << joint_trajectory_point.positions[6]
				);


		joint_trajectory_msgs.points.push_back(joint_trajectory_point);

		// Next position
		joint_trajectory_point.positions[6] += 1 * (M_PI/180);
		joint_trajectory_point.time_from_start = ros::Duration(0.05);
		joint_trajectory_msgs.points.push_back(joint_trajectory_point);

		joint_trajectory_publisher.publish(joint_trajectory_msgs);

		timer.sleep();
		ROS_INFO("Publish once");
		//ros::spinOnce();
	}

	return 0;
}
