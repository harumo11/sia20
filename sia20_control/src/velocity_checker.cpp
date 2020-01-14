#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "velocity_checker_node");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
	
	ros::Rate timer(40);
	while (ros::ok()) {
		const auto pose = move_group.getCurrentPose();
		std::cout << pose.pose.position.z << std::endl;
		timer.sleep();
	}
	return 0;
}
