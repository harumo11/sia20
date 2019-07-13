#include <iostream>
#include <fstream>
#include <sstream>
#include <fanda/String.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class RobotArm {
	public:
		RobotArm();
		~RobotArm();
		void start_trajectory(control_msgs::FollowJointTrajectoryGoal goal);
		control_msgs::FollowJointTrajectoryGoal arm_extension_trajectory(sensor_msgs::JointState joint_state, std::vector<double> timing_vec, std::vector<double> angle_vec, std::vector<double> angular_velocity_vec);
		// Return the current state of the action
		actionlib::SimpleClientGoalState get_state();

	private:
		// Action client for the JointTrajectoryAction used to trigger the arm movement action
		TrajectoryClient* trajctory_client;

};

/**
 * @brief Initialize the action client and wait for action server to come up
 */
RobotArm::RobotArm() {
	//tell the action client that we want to spin a thread by default	
	this->trajctory_client = new TrajectoryClient("/joint_trajectory_action", true);

	//wait for action server come up
	while (!this->trajctory_client->waitForServer(ros::Duration(1.0))) {
		ROS_INFO_STREAM("Waiting for the joint_trajctory_action server");
	}
};

/**
 * @brief Clean up the action client
 */
RobotArm::~RobotArm(){
	delete this->trajctory_client;
}

/**
 * @brief Send the command to start a given trajectory
 *
 * @param goal 
 */
void RobotArm::start_trajectory(control_msgs::FollowJointTrajectoryGoal goal){
	// When to start the trajectory 1s from now
	goal.trajectory.header.stamp = ros::Time::now();
	this->trajctory_client->sendGoal(goal);
}

/**
 * @brief Generate a simple trajectory with two waypoints, used as example.
 * Note that this trajectory contains two waypoints, joined together
 * as a single trajectory. Alternatively, each of these waypoints could 
 * be in its own trajectory - a trajectory can have one or more waypoints
 * depending on the desired application.
 *
 * @return 
 */
control_msgs::FollowJointTrajectoryGoal RobotArm::arm_extension_trajectory(sensor_msgs::JointState joint_state, std::vector<double> timing_vec, std::vector<double> angle_vec, std::vector<double> angular_velocity_vec){
	//our goal variable
	control_msgs::FollowJointTrajectoryGoal goal;

	//First the joint name, which apply to all waypoints
	goal.trajectory.joint_names.push_back("joint_s");
	goal.trajectory.joint_names.push_back("joint_l");
	goal.trajectory.joint_names.push_back("joint_e");
	goal.trajectory.joint_names.push_back("joint_u");
	goal.trajectory.joint_names.push_back("joint_r");
	goal.trajectory.joint_names.push_back("joint_b");
	goal.trajectory.joint_names.push_back("joint_t");
	ROS_INFO_STREAM("Goal trajctory gets joint names");

	// We will have 700 waypoints in this goal trajectory
	const int data_size = 700;
	goal.trajectory.points.resize(data_size);

	// Initial trajectory point which is recieved
	//const int initial_index = 0;
	for (int i = 0; i < data_size; i++) {
		// Positions
		// Set all trjectory as current position to initial joint angles
		goal.trajectory.points[i].positions.resize(7);
		goal.trajectory.points[i].positions[0] = joint_state.position[0];
		goal.trajectory.points[i].positions[1] = joint_state.position[1];
		goal.trajectory.points[i].positions[2] = joint_state.position[2];
		goal.trajectory.points[i].positions[3] = joint_state.position[3];
		goal.trajectory.points[i].positions[4] = joint_state.position[4];
		goal.trajectory.points[i].positions[5] = joint_state.position[5];
		goal.trajectory.points[i].positions[6] = joint_state.position[6];

		// Velocities                                 
		// Set all trjectory to initial joint angular velocities
		goal.trajectory.points[i].velocities.resize(7);
		for (int j = 0; j < 7; j++) {
			goal.trajectory.points[i].velocities[j] = 0;
		}
	}
	ROS_INFO_STREAM("Goal trajectory was initialized");

	// To be reached 1 second after starting alogn the trajectory
	goal.trajectory.header.stamp = ros::Time::now();
	//goal.trajectory.points[initial_index].time_from_start = ros::Duration(0);
	ROS_INFO_STREAM("Goal trajectory gets item from start of initial");

	// Prepare trajectory points
	const int joint_t = 6;
	for (int index = 0; index < data_size; index++) {
		// Set positions of joint_t from data file
		goal.trajectory.points[index].positions[joint_t] += angle_vec[index] * M_PI/180;
		ROS_INFO_STREAM("Debug angle : " << goal.trajectory.points[index].positions[joint_t] * 180/M_PI);
		// Set velocity of joint_t from data file
		goal.trajectory.points[index].velocities[joint_t] = angular_velocity_vec[index] * M_PI/180;
		// Set timing of moving from data file
		//goal.trajectory.points[index].time_from_start = ros::Duration(timing_vec[index-1]) + ros::Duration(index);
		goal.trajectory.points[index].time_from_start = ros::Duration(timing_vec[index]) + ros::Duration(index*0.05);
	}
	ROS_INFO_STREAM("Goal trajectory gets all data");

	// Display all trajectory
	ROS_INFO_STREAM("Target trajectory positions");
	for (int index = 0; index < data_size; index++) {
		for (int joint = 0; joint < 7; joint++) {
			std::cout << goal.trajectory.points[index].positions[joint] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	ROS_INFO_STREAM("Target trajectory velocity");
	for (int index = 0; index < data_size; index++) {
		for (int joint = 0; joint < 7; joint++) {
			std::cout << goal.trajectory.points[index].velocities[joint] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	ROS_INFO_STREAM("Target timing");
	for (int index = 0; index < data_size; index++) {
		std::cout << "[ " << index << " ] " << goal.trajectory.points[index].time_from_start << std::endl;
	}
	std::cout << std::endl;
	
	//We are done, return the goal
	return goal;
}


actionlib::SimpleClientGoalState RobotArm::get_state(){
	return this->trajctory_client->getState();
}

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState msgs);
		sensor_msgs::JointState joint_states;
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->joint_states = msgs;
	ROS_INFO_STREAM("Joint State : " << msgs.position[0] << " " << msgs.position[1] << " " << msgs.position[2] << " " << msgs.position[3] << " " << msgs.position[4] << " " << msgs.position[5] << " " << msgs.position[6]);
}

int main(int argc, char* argv[])
{
	// Read csv data
	std::ifstream file("/home/harumo/opt/Pouring/poursim_qrmax2_theta0_20_Sz_20.csv");
	if (file.is_open()) {
		ROS_INFO_STREAM("File is opend");
	}
	else {
		ROS_ERROR_STREAM("Can't open data file. Program aborted");
		return -1;
	}

	std::string one_line_sentence;
	std::vector<double> timing_vector;
	std::vector<double> angle_vector;
	std::vector<double> angular_velocity_vector;
	while (file >> one_line_sentence) {
		auto split_word = String::split(one_line_sentence, ',');
		timing_vector.push_back(std::stod(split_word[0]));
		angular_velocity_vector.push_back(std::stod(split_word[1]));
		angle_vector.push_back(std::stod(split_word[2]));
	}
	// Minus 20 degree from angle
	for (auto&& e : angle_vector){
		e -= 20;
		ROS_INFO_STREAM("Fixed angle : " << e);
	}
	
	// Display data file
	ROS_INFO_STREAM("Display data file [time, angle, angular velocity]");
	for (int i = 0; i < 700; i++) {
		ROS_INFO_STREAM(timing_vector[i] << " " << angle_vector[i] << " " << angular_velocity_vector[i]);
	}

	// Init the ROS node
	ros::init(argc, argv, "pouring_action_client");
	ros::NodeHandle node_handle;

	// Get initial joint state
	JointStateListener joint_state_listener;
	ros::Subscriber initial_joint_state_listener = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
	ros::Rate timer(10);
	for (int i = 0; i < 5; i++) {
		ros::spinOnce();
		timer.sleep();
	}

	// Create joint trajectory client
	RobotArm robot_arm;

	// Start the trajectory
	robot_arm.start_trajectory(robot_arm.arm_extension_trajectory(joint_state_listener.joint_states, timing_vector, angle_vector, angular_velocity_vector));

	// Wait for trajectory completion
	while (!robot_arm.get_state().isDone() && ros::ok()) {
		ROS_INFO_STREAM("I'm waiting robot arm finishing");
		timer.sleep();
	}

	return 0;
}
