// このプログラムはpose_follow_planner3から受け取った速度の命令値をsia20に
// 送るためのプログラムです．

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class JointStateListener {
	public:
		sensor_msgs::JointState data;
		void call_back(const sensor_msgs::JointState msgs) { this->data = msgs; };
};

class TargetDisplacementListener {
	public:
		geometry_msgs::Pose data;
		Eigen::VectorXd delta_x = Eigen::VectorXd::Zero(6);
		void call_back(const geometry_msgs::Pose msgs) { this->data = msgs; };
};

void set_joint_trajectry_config(trajectory_msgs::JointTrajectory& joint_trajectory_msgs){
	joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
	joint_trajectory_msgs.header.stamp = ros::Time::now();
}

void set_joint_trajectry_point_config(trajectory_msgs::JointTrajectoryPoint& joint_trajectory_point, const ros::Duration time_from_start){
	joint_trajectory_point.time_from_start = time_from_start; 
	joint_trajectory_point.velocities = {0, 0, 0, 0, 0, 0, 0};
}

Eigen::MatrixXd pseudo_inv(Eigen::MatrixXd J){
	return (J.transpose() * (J * J.transpose()).inverse());
}

int main(int argc, char* argv[])
{
	// ROS初期化
	ros::init(argc, argv, "pose_follow_executer_node");
	ros::AsyncSpinner spinner(3);
	spinner.start();
	ros::NodeHandle node_handle;
	const double control_cycle = 100;
	ros::Rate rate_timer(control_cycle);

	// publisher初期化
	ros::Publisher joint_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/joint_command", 1);

	// subscriber初期化
	JointStateListener joint_state_listener;
	TargetDisplacementListener target_displacement_listener;
	ros::Subscriber joint_subscriber = node_handle.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
	ros::Subscriber target_subscriber = node_handle.subscribe("/target_displacement", 1, &TargetDisplacementListener::call_back, &target_displacement_listener); 

	// moveit初期化
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	robot_model_loader::RobotModelLoader model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
	moveit::core::RobotState kinematic_state(kinematic_model);
	kinematic_state.setToDefaultValues();
	const moveit::core::JointModelGroup *joint_model_group_ptr;
	joint_model_group_ptr = kinematic_model->getJointModelGroup("manipulator");
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

	// JointTrajectoryをsia20に送る (初回)
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	set_joint_trajectry_config(joint_trajectory_msgs);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
	set_joint_trajectry_point_config(joint_trajectory_point, ros::Duration(0.0));
	joint_trajectory_point.positions = joint_state_listener.data.position;
	joint_trajectory_msgs.points.push_back(joint_trajectory_point);
	joint_trajectory_publisher.publish(joint_trajectory_msgs);
	
	const ros::Time t_start = ros::Time::now();
	ros::Time t_last = t_start;
	while (ros::ok()) {
		// dtを計算
		double dt = (ros::Time::now() - t_start).toSec();
		t_last = ros::Time::now();
		kinematic_state.setVariableValues(joint_state_listener.data);
		
		// Jacobianを取得
		const auto jacobian = kinematic_state.getJacobian(joint_model_group_ptr); // TODO getJacobian()はlink_tまでのもの，それともtool_t?
		const auto inv_jacobian = pseudo_inv(jacobian);

		// delta_qを計算
		const auto delta_q = inv_jacobian * target_displacement_listener.delta_x;
		ROS_DEBUG_STREAM("delta_q : " << delta_q);

		// JointTrajectoryPointに変換
		set_joint_trajectry_point_config(joint_trajectory_point, ros::Duration(ros::Time::now() - t_start));
		joint_trajectory_point.positions = joint_state_listener.data.position;
		for (int i = 0; i < 7; i++) {
			joint_trajectory_point.positions.at(i) = joint_state_listener.data.position.at(i) + delta_q(i);
			joint_trajectory_point.velocities.at(i) = delta_q(i);
		}

		// JointTrajectoryをpublish
		set_joint_trajectry_config(joint_trajectory_msgs);
		joint_trajectory_msgs.points.at(0) = joint_trajectory_point;
		joint_trajectory_publisher.publish(joint_trajectory_msgs);

		ROS_INFO("Publish once");
		rate_timer.sleep();
		ros::spinOnce();
	}

	return 0;
}
