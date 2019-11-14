#include <iostream>

#include <vector>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <eigen3/Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>

#include "quaternion_utility.hpp"
#include "target_velocity.hpp"

// This class prvides current joint angles of SIA20
class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_state;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_state = msgs;
}


int main(int argc, char* argv[])
{

	// ROS Initialize
	ros::init(argc, argv, "sequential_ik_solver");
	ros::NodeHandle node_handle;

	// AsyncSpinner Initialize 
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ROS_INFO_STREAM("Is spinner start : " << std::boolalpha << spinner.canStart());

	// Configuration
	const std::string PLANNING_GROUP = "manipulator";
	const unsigned int PUBLISH_CYCLE = 10;	// 50[Hz]

	// Make velocity subscriber
	TargetVelocity target_velocity;
	ros::Subscriber target_velocity_subscriber = node_handle.subscribe("/target_velocity", 1, &TargetVelocity::call_back, &target_velocity);

	// Make Joint State Listener
	JointStateListener joint_state_listener;
	ros::Subscriber current_joint_listener = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);

	// Make joint_trajectory_publisher
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);

	// MoveIt! setting
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setMaxVelocityScalingFactor(1);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	auto kinematic_model = robot_model_loader.getModel();
	auto kinematic_state = new robot_state::RobotState(kinematic_model);
	//kinematic_state->setToDefaultValues();
	auto joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);

	// Timer setting
	ros::Rate timer(10);

	while (ros::ok()) {
		
		// Get current pose of link_t
		geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose("link_t");

		// Convert from quaternion to RPY
		Eigen::Quaterniond current_quat(current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
		Eigen::Vector3d current_rpy = q2rpy(current_quat);

		// current_pose + velocity_command
		// translation
		geometry_msgs::Pose next_pose = current_pose.pose;
		next_pose.position.x += target_velocity.get().linear.x;
		//next_pose.position.x += 0.01;
		next_pose.position.y += target_velocity.get().linear.y;
		next_pose.position.z += target_velocity.get().linear.z;

		// orientation
		// add in rpy field
		current_rpy[0] += target_velocity.get().angular.x;
		current_rpy[1] += target_velocity.get().angular.y;
		current_rpy[2] += target_velocity.get().angular.z;
		// convert to quaternion field
		Eigen::Quaterniond next_quaternion = rpy2q(current_rpy);
		next_pose.orientation.x = next_quaternion.x();
		next_pose.orientation.y = next_quaternion.y();
		next_pose.orientation.z = next_quaternion.z();
		next_pose.orientation.w = next_quaternion.w();



		// calculation ik
		const double timeout = 0.1;
		bool found_ik = kinematic_state->setFromIK(joint_model_group, next_pose, timeout);
		std::vector<double> joint_values;

		// get curren joint states
		for (auto e : joint_state_listener.joint_state.position){
			joint_values.push_back(e);
		}
		std::cout << joint_state_listener.joint_state << std::endl;

		if (found_ik) {
			
			// make message for publishing
			trajectory_msgs::JointTrajectory joint_trajectory_messages;
			joint_trajectory_messages.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
			joint_trajectory_messages.header.stamp = ros::Time::now();
			trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
			joint_trajectory_points.time_from_start = ros::Duration(0.05);

			//kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			//for (auto&& joint : joint_values){
			//	joint_trajectory_points.positions.push_back(joint);
			//	joint_trajectory_points.velocities.push_back(0);
			//}
			//ROS_INFO_STREAM("Joints : " << joint_trajectory_points);

			for (auto e : joint_state_listener.joint_state.position){
				joint_trajectory_points.positions.push_back(e);
				joint_trajectory_points.velocities.push_back(0);
			}

			// publish
			joint_trajectory_messages.points.push_back(joint_trajectory_points);

			//----------------------------------------------------------
			joint_trajectory_points.positions.at(1) += 0.005;
			joint_trajectory_points.time_from_start = ros::Duration(0.1);
			joint_trajectory_messages.points.push_back(joint_trajectory_points);
			joint_trajectory_publisher.publish(joint_trajectory_messages);
			ROS_INFO_STREAM("Publish once");
		}
		else {
			ROS_WARN_STREAM("Could not find IK");
		}

		// Receive new target velocity
		timer.sleep();

	}
	
	return 0;
}
