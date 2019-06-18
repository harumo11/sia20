#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>

class TargetVelocity : public geometry_msgs::Twist
{
public:
	void call_back(const geometry_msgs::TwistConstPtr target_velocity_data);
	void print_data();
	geometry_msgs::Twist get();
};

void TargetVelocity::call_back(const geometry_msgs::TwistConstPtr target_velocity_data){
	// Move listened data to memeber 
	this->linear = target_velocity_data->linear;
	this->angular = target_velocity_data->angular;
	this->print_data();
}

geometry_msgs::Twist TargetVelocity::get(){
	geometry_msgs::Twist twist;
	twist.linear = this->linear;
	twist.angular = this->angular;

	return twist;
}

void TargetVelocity::print_data(){
	ROS_INFO_STREAM("Target velocity : " << "x : " << this->linear.x  << "\t"
	                                     << "y : " << this->linear.y  << "\t"
	                                     << "z : " << this->linear.z  << "\t"
	                                     << "r : " << this->angular.x << "\t"
	                                     << "p : " << this->angular.y << "\t"
	                                     << "w : " << this->angular.z << "\t"
										 );
}

geometry_msgs::PoseStamped calc_next_position(geometry_msgs::PoseStamped x, geometry_msgs::Twist velocity){
	// Add next position
	x.pose.position.x += velocity.linear.x;
	x.pose.position.y += velocity.linear.y;
	x.pose.position.z += velocity.linear.z;

	// Convert angler
	tf2::Quaternion quat1;
	tf2::Quaternion quat2;
	quat1.setRPY(velocity.angular.x, velocity.angular.y, velocity.angular.z);
	quat2.setValue(x.pose.orientation.x, x.pose.orientation.y, x.pose.orientation.z, x.pose.orientation.w);

	// Add next orientation
	tf2::Quaternion quat3 = quat1 + quat2;
	x.pose.orientation.x = quat3.x();
	x.pose.orientation.y = quat3.y();
	x.pose.orientation.z = quat3.z();
	x.pose.orientation.w = quat3.w();

	return x;
}


int main(int argc, char* argv[])
{

	// ROS Initialize
	ros::init(argc, argv, "sequential_ik_solver");
	ros::NodeHandle node_handle;

	// AsyncSpinner Initialize 
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO_STREAM("Is spinner start : " << std::boolalpha << spinner.canStart());

	// Configuration
	const std::string PLANNING_GROUP = "sia20_arm";
	const unsigned int PUBLISH_CYCLE = 50;	// 50[Hz]

	// MoveIt! Initialize
	ros::Rate timer(PUBLISH_CYCLE);
	const double timeout = 0.015;
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	ROS_INFO_STREAM("End effector name : " << move_group.getEndEffector());
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame : %s", kinematic_model->getModelFrame().c_str());
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_goup = kinematic_model->getJointModelGroup("sia20_arm");
	auto joint_names = joint_model_goup->getVariableNames();
	int name_itr = 0;
	for (auto&& e : joint_names){
		std::cout << "Joint" << name_itr << "\t" << e << std::endl;
	}

	// Listener Initialize
	TargetVelocity target_velocity;
	ros::Subscriber subscriber = node_handle.subscribe("target_velocity", 1, &TargetVelocity::call_back, &target_velocity);

	// Publisher Inistialize
	ros::Publisher publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/sia20/sia20_joint_controller/command", 1);
	ros::Publisher publisher_debug = node_handle.advertise<geometry_msgs::PoseStamped>("/sia20/target_position", 1);


	while (ros::ok()) {
		// Get current pose x
		ROS_INFO_STREAM(move_group.getCurrentPose());
		auto x = move_group.getCurrentPose();
		auto x_next_ = calc_next_position(x, target_velocity.get());
		publisher_debug.publish(x_next_);
		geometry_msgs::Pose x_next;
		x_next = x_next_.pose;

		// Solve IK
		bool found_ik = kinematic_state->setFromDiffIK(joint_model_goup, target_velocity.get(), "link_t", timeout);
		//bool found_ik = kinematic_state->setFromIK(joint_model_goup, x_next);


		// Can solve IK in successfuly
		if (!found_ik)
		{
			ROS_ERROR_STREAM("Can't solve inverse kinematics");
		}
		else
		{
			// Make jointTrajectory message
			trajectory_msgs::JointTrajectory joint_trajectory_msgs;
			joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
			std::vector<double> joint_values;
			trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
			kinematic_state->copyJointGroupPositions(joint_model_goup, joint_values);
			for (auto&& e : joint_values){
				joint_trajectory_points.positions.push_back(e);
				joint_trajectory_points.time_from_start = ros::Duration(0.001);
			}

			// Publish JointTrajctory
			joint_trajectory_msgs.points.push_back(joint_trajectory_points);
			publisher.publish(joint_trajectory_msgs);
		}

		timer.sleep();
	}

	spinner.stop();
	return 0;
}
