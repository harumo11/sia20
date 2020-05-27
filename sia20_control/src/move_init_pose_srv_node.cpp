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
	//old init pose
	//const std::vector<double> init_joint_values = {-0.7081537246704102, 0.396482914686203, -0.0008522116113454103, -0.8483936786651611, -1.0959001779556274, -0.66163170337677, 1.2956724166870117};
	//x axis visual servo init pose
	//long
	//const std::vector<double> init_joint_values = {-0.42932716012, 0.0807896628976, 0.000443150027422, -1.30204296112, -1.30793881416, -0.30667462945, 1.5433652401};
	//short
	const std::vector<double> init_joint_values = {-0.42922487855, 0.110787510872, 0.000528371194378, -1.78560483456, -2.51194667816, -0.517771661282, 2.8249912262};
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
