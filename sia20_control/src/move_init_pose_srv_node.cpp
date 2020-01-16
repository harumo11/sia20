#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>

bool should_init = false;

void call_back(sensor_msgs::Joy msgs){
	if (msgs.buttons[3] == 1) {	// viveコントローラの持ち手にある楕円形のボタンのどちらかが押されたら
		ROS_INFO_STREAM("Move init pose");
		should_init = true;
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "move_init_pose_srv_node");
	ros::NodeHandle node;
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
	ros::Subscriber subs = node.subscribe("/vive/controller_LHR_066549FF/joy", 1, call_back);
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ROS_INFO_STREAM("Init pose node starts");

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const std::vector<double> init_joint_values = {1.61967921257, -0.316306859255, -1.86695694923, -1.14934372902, -0.374367237091, -0.273443460464, 0.000240624460275};
	//const std::vector<double> init_joint_values = {2.27785921097 -0.441973984241 -2.64661121368 -0.966646552086 -0.884180426598 -0.51678442955 -2.74567556381};

	ros::Rate timer(10);
	while (ros::ok()) {
		if (should_init == true) {
			move_group.setJointValueTarget(init_joint_values);
			move_group.move();
			should_init = false;
			ros::Duration(2.0).sleep();
		}
		timer.sleep();
	}

	return 0;
}
