#include <iostream>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
	ros::Rate timer(1);

	while (ros::ok()) {
		// get current pose (quaternion)
		geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose("link_t");
		ROS_INFO_STREAM("current pose : " << current_pose);
		debug_publisher_current_pose.publish(current_pose);

		// get current pose (homogenious matrix) aka Forward kinematics
		auto tip_state = kinematic_state->getGlobalLinkTransform("link_t");
		std::cout << "[translation] : " << tip_state.translation() << std::endl;
		std::cout << "[rotation] : " << tip_state.rotation() << std::endl;

		// convert from quaternion to RPY
		tf2::Quaternion quat(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		std::cout << "roll : " << roll << " pitch : " << pitch << " yaw : " << yaw << std::endl;

		// Publish next pose
		current_pose.pose.position.z -= 0.1;
		debug_publisher_next_pose.publish(current_pose);

		// calc ik
		geometry_msgs::Pose next_pose = current_pose.pose;
		double timeout = 0.1;
		bool found_ik = kinematic_state->setFromIK(joint_model_group, next_pose, timeout);
		std::vector<double> joint_values;
		if (found_ik) {
			// make message for publishing
			trajectory_msgs::JointTrajectory joint_trajectory_messages;
			joint_trajectory_messages.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
			joint_trajectory_messages.header.stamp = ros::Time::now();
			trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
			joint_trajectory_points.time_from_start = ros::Duration(0.0001);

			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			for (auto&& joint : joint_values){
				joint_trajectory_points.positions.push_back(joint);
			}
			ROS_INFO_STREAM("joints : " << joint_trajectory_points);

			// publish
			joint_trajectory_messages.points.push_back(joint_trajectory_points);
			joint_trajectory_publisher.publish(joint_trajectory_messages);
			ROS_INFO_STREAM("Publish once");

		}
		else {
			ROS_WARN_STREAM("Couldn't find IK");
		}

		timer.sleep();
	}


	return 0;
}
