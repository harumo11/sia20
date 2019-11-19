#include <iostream>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/WorldState.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "/home/harumo/catkin_ws/devel/include/gazebo_ros_link_attacher/Attach.h"
#include "quaternion_utility.hpp"

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

	ros::Rate timer(5);
	std_msgs::Float64 torque;
	if (is_hand_open == true){
		torque.data = -1000;
		ROS_INFO_STREAM("Hand tries to close");
	}
	else {
		torque.data = 1000;
		ROS_INFO_STREAM("Hand tries to open");
	}
	is_hand_open = !is_hand_open;
	std::cout << "is_hand_open" << std::boolalpha << is_hand_open << std::endl;

	for (int i = 0; i < 2; i++)
	{
		right_hand.publish(torque);
		left_hand.publish(torque);
		timer.sleep();
		ROS_INFO_STREAM("Hand Action");
	}
}

void save_log(std::ofstream& file, std::vector<geometry_msgs::Pose> observation, bool is_hand_open, std::vector<double> a){

	std::stringstream one_line;

	// gazebo pose
	for (auto e : observation){
		//convert from quaternion to rpy
		Eigen::Quaterniond quat(e.orientation.w, e.orientation.x, e.orientation.y, e.orientation.z);
		auto rpy = q2rpy(quat);
		one_line << e.position.x << "," << e.position.y << "," << e.position.z << "," << rpy[0] << "," << rpy[1] <<  "," << rpy[2] << ",";
	}

	// hand
	one_line << is_hand_open + 0 << ",";

	// joint values
	for (auto joint_value : a){
		one_line << joint_value << ",";
	}
	
	if (file.is_open()){
		file << one_line.str() << std::endl;
		std::cout << one_line.str() << std::endl;
	}
	else
	{
		ROS_WARN_STREAM("cannot save pose data");
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

	//sを観測
	auto link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("1. Observing s");
	for (auto e : link_pose){
		ROS_INFO_STREAM(e);
	}

	//publisher
	ros::Publisher r_hand_controller = node_handler.advertise<std_msgs::Float64>("/sia20/sia20_wrist_to_r_controller/command", 1);
	ros::Publisher l_hand_controller = node_handler.advertise<std_msgs::Float64>("/sia20/sia20_wrist_to_l_controller/command", 1);

	//hand config
	bool is_hand_open = false;
	//ハンドを開く
	hand_action(is_hand_open, r_hand_controller, l_hand_controller);
	ROS_INFO_STREAM("2. Opening hand");

	//moveit
	static const std::string PLANNING_GROUP = "sia20_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	auto joint_vec = move_group.getCurrentJointValues();
	for (auto e : joint_vec){
		std::cout << e << std::endl;
	}

	//file
	std::ofstream log_file("/home/harumo/catkin_ws/src/sia20/sia20_control/data/log.csv", std::ios_base::app);
	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("3. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("4. Observing s");

	//move1
	geometry_msgs::Pose tool_pose = link_pose.at(2);
	tool_pose.orientation.w = 0.5444;
	tool_pose.orientation.x = -0.4362;
	tool_pose.orientation.y = -0.4733;
	tool_pose.orientation.z = 0.5377;
	tool_pose.position.z -= 0.85;
	//tool_pose.position.z = tool_pose.position.z + 0.1;
	move_group.setPoseTarget(tool_pose);
	moveit::planning_interface::MoveGroupInterface::Plan plan_to_tool;
	ROS_INFO_STREAM("5.0. Making the plan to tool1");
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to tool");
	}
	else {
		ROS_WARN_STREAM("Can't find path to tool");
	}
	//移動1
	move_group.move();
	ROS_INFO_STREAM("6.0. Moving");

	//move2
	tool_pose.orientation.w = 0.5444;
	tool_pose.orientation.x = -0.4362;
	tool_pose.orientation.y = -0.4733;
	tool_pose.orientation.z = 0.5377;
	tool_pose.position.z -= 0.05;
	//tool_pose.position.z = tool_pose.position.z + 0.1;
	move_group.setPoseTarget(tool_pose);
	ROS_INFO_STREAM("5.5. Making the plan to tool1");
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to tool");
	}
	else {
		ROS_WARN_STREAM("Can't find path to tool");
	}
	//移動
	move_group.move();
	ROS_INFO_STREAM("6.5. Moving");
	
	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("7. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("8. Observing s");

	//ハンドを閉じる
	hand_action(is_hand_open, r_hand_controller, l_hand_controller);
	ROS_INFO_STREAM("9. Opening hand");
	//ハンドとtoolを固定する
	ros::ServiceClient hand_client = node_handler.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
	hand_client.waitForExistence();
	gazebo_ros_link_attacher::Attach attach_srv;
	attach_srv.request.model_name_1 = "sia20";
	attach_srv.request.link_name_1  = "link_t";
	attach_srv.request.model_name_2 = "tool1";
	attach_srv.request.link_name_2  = "link";
	if(hand_client.call(attach_srv)){
		ROS_INFO_STREAM("tool is attached to hand");
	}
	else{
		ROS_WARN_STREAM("tool can't be attached to hand");
	}

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("10. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("10.0. Observing s");

	//move2
	tool_pose.orientation.w = 0.5444;
	tool_pose.orientation.x = -0.4362;
	tool_pose.orientation.y = -0.4733;
	tool_pose.orientation.z = 0.5377;
	tool_pose.position.z += 0.1;
	move_group.setPoseTarget(tool_pose);
	ROS_INFO_STREAM("10.5. Making the plan to tool1");
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to tool");
	}
	else {
		ROS_WARN_STREAM("Can't find path to tool");
	}
	//移動
	move_group.move();
	ROS_INFO_STREAM("10.7.5. Moving");

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("10.9. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("11. Observing s");

	//target上空に移動するプランを立てる
	auto target_pose = link_pose[1];
	target_pose.position.z -= 1.0;
	target_pose.position.z += 0.5;
	target_pose.orientation.x = 0.024;
	target_pose.orientation.y = -0.722;
	target_pose.orientation.z = -0.01;
	target_pose.orientation.w = 0.690;
	move_group.setPoseTarget(target_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to target above");
	}
	else {
		ROS_WARN_STREAM("Can't find path to target above");
	}

	//移動
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("11.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("11. Observing s");

	//target側面にtoolが移動するプランを立てる
	geometry_msgs::Pose side_pose;
	side_pose.position.x = 0.627;
	side_pose.position.y = 0.548;
	side_pose.position.z = 1.747 - 1;
	side_pose.orientation.x = 0.074;
	side_pose.orientation.y = -0.015;
	side_pose.orientation.z = -0.070;
	side_pose.orientation.w = 0.994;
	move_group.setPoseTarget(side_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to target side");
	}
	else {
		ROS_WARN_STREAM("Can't find path to target side");
	}

	//移動
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	//tool1がgoalへ移動するようなプランを作る
	auto slide_pose = side_pose;
	slide_pose.position.y -= 0.2;
	move_group.setPoseTarget(slide_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to slide");
	}
	else {
		ROS_WARN_STREAM("Can't find path to slide");
	}
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	slide_pose.position.y -= 0.2;
	move_group.setPoseTarget(slide_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to slide");
	}
	else {
		ROS_WARN_STREAM("Can't find path to slide");
	}
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	slide_pose.position.y -= 0.2;
	move_group.setPoseTarget(slide_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to slide");
	}
	else {
		ROS_WARN_STREAM("Can't find path to slide");
	}
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	slide_pose.position.y -= 0.2;
	move_group.setPoseTarget(slide_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to slide");
	}
	else {
		ROS_WARN_STREAM("Can't find path to slide");
	}
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	slide_pose.position.x = 0.578;
	slide_pose.position.y = -0.44;
	slide_pose.position.z = 0.759;
	slide_pose.orientation.x = 0.075;
	slide_pose.orientation.y = 0.009;
	slide_pose.orientation.z = -0.391;
	slide_pose.orientation.w = 0.916;
	move_group.setPoseTarget(slide_pose);
	if (move_group.plan(plan_to_tool) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_INFO_STREAM("Find path to slide");
	}
	else {
		ROS_WARN_STREAM("Can't find path to slide");
	}
	move_group.move();

	//sとaを保存
	save_log(log_file, link_pose, is_hand_open, joint_vec);
	ROS_INFO_STREAM("17.. s and a are saved");

	//sを観測
	link_pose = observe(gazebo_client);
	ROS_INFO_STREAM("18. Observing s");

	//終了処理，ハンドを開く
	hand_action(is_hand_open, r_hand_controller, l_hand_controller);
	//ほうきをはなす
	ros::ServiceClient hand_client_detach = node_handler.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
	hand_client_detach.waitForExistence();
	gazebo_ros_link_attacher::Attach detach_srv;
	detach_srv.request.model_name_1 = "sia20";
	detach_srv.request.link_name_1  = "link_t";
	detach_srv.request.model_name_2 = "tool1";
	detach_srv.request.link_name_2  = "link";
	if(hand_client_detach.call(detach_srv)){
		ROS_INFO_STREAM("tool is detached from hand");
	}
	else{
		ROS_WARN_STREAM("tool can't be detached from hand");
	}

	//全ての関節を０にする．
	move_group.setNamedTarget("virtical");
	move_group.move();


	//gazeboにシュミレーションをリセットするrpcを投げる
	std_srvs::Empty gz_reset_srv;
	if(ros::service::call("/gazebo/reset_world", gz_reset_srv)){
		ROS_INFO_STREAM("reset model pose");
	}
	else{
		ROS_WARN_STREAM("can't reset model pose");
	}
	return 0;
}
