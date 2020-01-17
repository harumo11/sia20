// このプログラムはpose_follow_planner3から受け取った速度の命令値をsia20に
// 送るためのプログラムです．

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_state;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_state = msgs;
}

class TargetDisplacementListener {
	public:
		void call_back(const geometry_msgs::Pose msgs);
		geometry_msgs::Pose data;
};

void TargetDisplacementListener::call_back(const geometry_msgs::Pose msgs){
	this->data = msgs;
}

void set_joint_trajectry_config(trajectory_msgs::JointTrajectory& joint_trajectory_msgs){
	//set joint name
	joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
	//set stamp
	joint_trajectory_msgs.header.stamp = ros::Time::now();
}

void set_joint_trajectry_point_config(trajectory_msgs::JointTrajectoryPoint& joint_trajectory_point, const ros::Duration time_from_start){
	joint_trajectory_point.time_from_start = time_from_start; 
	joint_trajectory_point.velocities = {0, 0, 0, 0, 0, 0, 0};
}

Eigen::MatrixXd p_inv(Eigen::MatrixXd J){
	return (J.transpose() * (J * J.transpose()).inverse());
}

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "joint_trajectory_publisher");
	ros::AsyncSpinner spinner(3);
	spinner.start();

	// create ros node handle
	ros::NodeHandle node_handle;

	// make publisher. NOTE use same topic name between Publisher and Subscriber.
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/joint_command", 1);

	// make subscriber. 
	JointStateListener joint_state_listener;
	TargetDisplacementListener target_displacement_listener;
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
	ros::Subscriber target_subscriber = node_handle.subscribe("/target_displacement", 1, &TargetDisplacementListener::call_back, &target_displacement_listener); 

	// moveit 初期化
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	robot_model_loader::RobotModelLoader model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
	moveit::core::RobotState kinematic_state(kinematic_model);
	kinematic_state.setToDefaultValues();
	const moveit::core::JointModelGroup *joint_model_group_ptr;
	joint_model_group_ptr = kinematic_model->getJointModelGroup("manipulator");
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

	// make loop timer
	//ros::Rate rate_timer(10);
	const double control_cycle = 100;
	ros::Rate rate_timer(control_cycle);

	int counter = 0;
	//const double T = 0.1;
	const double T = 1.0 / control_cycle;
	std::cout << "T : " << T << std::endl;

	// subscribe current position (first time)
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	set_joint_trajectry_config(joint_trajectory_msgs);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
	set_joint_trajectry_point_config(joint_trajectory_point, ros::Duration(0.0));
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
