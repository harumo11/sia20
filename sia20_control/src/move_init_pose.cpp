#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "move_init_pose_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	if (spinner.canStart()) {
		ROS_INFO_STREAM("AsyncSpinner starts");
	}
	else {
		ROS_ERROR_STREAM("AsyncSpinner can not start");
	}

	static const std::string PLANNING_GROUP = "manipulator";
	ROS_INFO_STREAM("Planning group : " << PLANNING_GROUP);

	// move_group setup 
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	ROS_INFO_STREAM(move_group.getCurrentPose());

	geometry_msgs::Pose target_pose;



	return 0;
}
