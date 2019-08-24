#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//sを観測する関数(ハンド以外)
std::vector<geometry_msgs::Pose> observe(ros::ServiceClient gazebo_client){

	//request
	gazebo_msgs::GetLinkState gazebo_srv;
	gazebo_srv.request.reference_frame = "world";

	geometry_msgs::Pose sia20_pose;
	geometry_msgs::Pose target_pose;
	geometry_msgs::Pose tool_pose;

	gazebo_srv.request.link_name = "sia20::link_t";
	if (gazebo_client.call(gazebo_srv)) {
		ROS_INFO_STREAM(gazebo_srv.request.link_name);
		ROS_INFO_STREAM(gazebo_srv.response.link_state.pose);
		sia20_pose = gazebo_srv.response.link_state.pose;
	}
	else {
		ROS_INFO_STREAM("Faild sia20::link_t");
	}

	gazebo_srv.request.link_name = "target::link";
	if (gazebo_client.call(gazebo_srv)) {
		ROS_INFO_STREAM(gazebo_srv.request.link_name);
		ROS_INFO_STREAM(gazebo_srv.response.link_state.pose);
		target_pose = gazebo_srv.response.link_state.pose;
	}
	else {
		ROS_INFO_STREAM("Failed target::link");
	}

	gazebo_srv.request.link_name = "tool1::link";
	if (gazebo_client.call(gazebo_srv)) {
		ROS_INFO_STREAM(gazebo_srv.request.link_name);
		ROS_INFO_STREAM(gazebo_srv.response.link_state.pose);
		tool_pose = gazebo_srv.response.link_state.pose;
	}
	else {
		ROS_INFO_STREAM("Faild tool1::link");
	}

	return {sia20_pose, target_pose, tool_pose};
}

void hand_action(bool& is_hand_open, ros::Publisher right_hand, ros::Publisher left_hand){

	ros::Rate timer(1);
	std_msgs::Float64 torque;
	if (is_hand_open == true){
		torque.data = -100;
		ROS_INFO_STREAM("Hand tries to close");
	}
	else {
		torque.data = 100;
		ROS_INFO_STREAM("Hand tries to open");
	}
	is_hand_open = !is_hand_open;
	std::cout << "is_hand_open" << std::boolalpha << is_hand_open << std::endl;

	for (int i = 0; i < 3; i++)
	{
		right_hand.publish(torque);
		left_hand.publish(torque);
		timer.sleep();
		ROS_INFO_STREAM("Hand Action");
	}
}

int main(int argc, char* argv[])
{
	//ros init
	ros::init(argc, argv, "sim_manager");
	ROS_INFO_STREAM("Init");

	//NodeHandler
	ros::NodeHandle node_handler;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO_STREAM("NodeHandle and Spinner start");

	//client
	ros::ServiceClient gazebo_client = node_handler.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
	gazebo_client.waitForExistence();
	ROS_INFO_STREAM("Gazebo Client starts");

	//publisher
	ros::Publisher r_hand_controller = node_handler.advertise<std_msgs::Float64>("/sia20/sia20_wrist_to_r_controller/command", 1);
	ros::Publisher l_hand_controller = node_handler.advertise<std_msgs::Float64>("/sia20/sia20_wrist_to_l_controller/command", 1);

	//hand config
	bool is_hand_open = false;
	hand_action(is_hand_open, r_hand_controller, l_hand_controller);
	hand_action(is_hand_open, r_hand_controller, l_hand_controller);
	
	return 0;
}
