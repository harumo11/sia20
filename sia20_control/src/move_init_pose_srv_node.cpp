#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "move_init_pose_srv_node");
	ros::NodeHandle node;
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO_STREAM("Init pose node starts");

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const std::vector<double> init_joint_values = {-0.5866283774375916, 0.27649152278900146, 0.0, -0.9963206052780151, -1.0167710781097412, -0.5848567485809326, 0.8310868144035339};
	// make robot be enable
	std_srvs::Trigger robot_enable_triger;
	ros::service::call("/robot_enable", robot_enable_triger);
	ros::Duration(1.0).sleep();

	move_group.setMaxVelocityScalingFactor(0.4);
	move_group.setJointValueTarget(init_joint_values);
	move_group.move();
	ros::Duration(2.0).sleep();

	return 0;
}
