#include <iostream>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

Eigen::Vector3d rad2deg(Eigen::Vector3d rad){
	return (108.0 / M_PI) * rad;
}

Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
	Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
					     * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
	 				     * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
	return q;
}

Eigen::Vector3d q2rpy(Eigen::Quaterniond q){
	return q.matrix().eulerAngles(0,1,2);
}

Eigen::Vector3d q2rpy2deg(Eigen::Quaterniond q){
	return rad2deg(q2rpy(q));
}


int main(int argc, char* argv[])
{
	// config
	const std::string planning_group = "sia20_arm";

	ros::init(argc, argv, "test_ik_node");
	ros::NodeHandle node_handler;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher debug_publisher_current_pose = node_handler.advertise<geometry_msgs::PoseStamped>("/sia20/debug/current_pose", 1);
	ros::Publisher debug_publisher_next_pose = node_handler.advertise<geometry_msgs::PoseStamped>("/sia20/debug/next_pose", 1);
	ros::Publisher joint_trajectory_publisher = node_handler.advertise<trajectory_msgs::JointTrajectory>("/sia20/sia20_joint_controller/command", 1);

	moveit::planning_interface::MoveGroupInterface move_group(planning_group);
	move_group.setMaxVelocityScalingFactor(0.1);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	auto kinematic_model = robot_model_loader.getModel();
	auto kinematic_state = new robot_state::RobotState(kinematic_model);
	kinematic_state->setToDefaultValues();
	auto joint_model_group = kinematic_model->getJointModelGroup(planning_group);

	// timer
	ros::Rate timer(10);

	// iterration
	int itr = 0;

	while (ros::ok()) {
		// get current pose (quaternion)
		geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose("link_t");
		//ROS_INFO_STREAM("current pose : " << current_pose);
		debug_publisher_current_pose.publish(current_pose);

		// get current pose (homogenious matrix) aka Forward kinematics
		auto tip_state = kinematic_state->getGlobalLinkTransform("link_t");
		std::cout << "[translation] : " << tip_state.translation() << std::endl;
		std::cout << "[rotation] : " << tip_state.rotation() << std::endl;

		std::vector<double> joint_values;
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(auto joint : joint_values){
			std::cout << joint << std::endl;
		}
	}

	return 0;
}
