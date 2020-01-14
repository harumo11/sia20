// This program is under construction!!
//
#include <iostream>
#include <ros/ros.h>
#include <memory>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

class RobotArm {
	public:
		RobotArm();
		~RobotArm();
		void start_trajectory(control_msgs::FollowJointTrajectoryGoal target_trajectory);
		actionlib::SimpleClientGoalState get_state();
		control_msgs::FollowJointTrajectoryGoal prepare_msgs(const int message_size = 1);
		sensor_msgs::JointState get_joint_state();

	private:
		std::shared_ptr<TrajectoryClient> trajectory_client_ptr = std::make_shared<TrajectoryClient>("/", true);

};

RobotArm::RobotArm() {
	//wait for action server come up
	while (!this->trajectory_client_ptr->waitForServer(ros::Duration(1.0))) {
		ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
	}
}

actionlib::SimpleClientGoalState RobotArm::get_state() {
	return this->trajectory_client_ptr->getState();
}

RobotArm::~RobotArm(){
	
}

control_msgs::FollowJointTrajectoryGoal prepare_msgs(const int message_size){
	control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msgs;

	// First the jont name, which apply to all waypoints
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_s");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_l");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_e");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_u");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_r");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_b");
	follow_joint_trajectory_msgs.trajectory.joint_names.push_back("joint_t");
	ROS_INFO_STREAM("Fill trajectory message with joint name");

	follow_joint_trajectory_msgs.trajectory.points.resize(message_size);

	for (int i = 0; i < message_size; i++) {
		follow_joint_trajectory_msgs.trajectory.points[i].positions.resize(7);
	}

	return follow_joint_trajectory_msgs;
}

void RobotArm::start_trajectory(control_msgs::FollowJointTrajectoryGoal target_trajectory){
	//When to start the trajectory: from now
	target_trajectory.trajectory.header.stamp = ros::Time::now();
	this->trajectory_client_ptr->sendGoal(target_trajectory);
}

int main(int argc, char const* argv[])
{
	
	return 0;
}
