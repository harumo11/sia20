#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "joint_trajectory_publisher");

	// create ros node handle
	ros::NodeHandle node_handle;

	// make publisher. NOTE use same topic name between Publisher and Subscriber.
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/sia20/sia20_joint_controller/command", 1);

	// make loop timer
	ros::Rate rate_timer(1);

	int counter = 0;
	double step_size = 0.03;

	while (ros::ok()) {
		
		// make messages for publishing
		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
		joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", 
											 "joint_u", "joint_r", "joint_b", "joint_t"};

		trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
		for (int i = 0; i < 7; i++){
			joint_trajectory_point.positions.push_back(counter * step_size);
			joint_trajectory_point.time_from_start = ros::Duration(0.1);
		}

		joint_trajectory_msgs.points.push_back(joint_trajectory_point);

		joint_trajectory_publisher.publish(joint_trajectory_msgs);

		counter++;

		ROS_INFO("Publish once");

		rate_timer.sleep();
	}

	return 0;
}
