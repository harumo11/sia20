#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief getting differential position via ROS topic.
 *
 * @param diff_position_ptr represents displacement position of SIA20's hand tip. 
 */
//void get_diff_position_cb(const geometry_msgs::TwistConstPtr diff_position_ptr){
//	std::lock_guard<std::mutex> lock(m);	//locking starts
//
//	ROS_DEBUG_STREAM("Received position displacement\t" << "x: " << diff_position_ptr->linear.x << "\t"
//													    << "y: " << diff_position_ptr->linear.y << "\t"
//													    << "z: " << diff_position_ptr->linear.z << "\t"
//													    << "r: " << diff_position_ptr->angular.x << "\t"
//													    << "p: " << diff_position_ptr->angular.y << "\t"
//													    << "w: " << diff_position_ptr->angular.z << "\t"
//														);
//}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sequential_ik_solver");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(3);
	spinner.start();
	std::cout << "Is start spinner? " << std::boolalpha << spinner.canStart() << std::endl;
	const std::string PLANNING_GROUP = "sia20_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	//move_group.setEndEffectorLink("link_t");
	std::cout << "end effector name is " << move_group.getEndEffector() << std::endl;

	while (ros::ok()) {
		std::cout << move_group.getCurrentPose() << std::endl;
	}

	return 0;
}
