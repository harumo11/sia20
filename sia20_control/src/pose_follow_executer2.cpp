// このプログラムはpose_follow_planner3から受け取った速度の命令値をsia20に
// 送るためのプログラムです．

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
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
		void call_back(const geometry_msgs::Pose msgs);
};

void TargetDisplacementListener::call_back(const geometry_msgs::Pose msgs){
	//位置の変位をコピー
	this->delta_x(0) = msgs.position.x / 0.01;
	this->delta_x(1) = msgs.position.y / 0.01; 
	this->delta_x(2) = msgs.position.z / 0.01;

	//メッセージの姿勢がquaternionで表されているので，roll, pitch, yawになおす．
	Eigen::Quaterniond quat(msgs.orientation.w, msgs.orientation.x, msgs.orientation.y, msgs.orientation.z);
	
	//姿勢の変位をコピー
	const auto rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);
	this->delta_x(3) = rpy(0) / 0.01;
	this->delta_x(4) = rpy(1) / 0.01;
	this->delta_x(5) = rpy(2) / 0.01;

	std::cout << "delta_x " << std::endl;
	std::cout << delta_x << std::endl;
}

void set_joint_trajectry_config(trajectory_msgs::JointTrajectory& joint_trajectory_msgs){
	joint_trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
	joint_trajectory_msgs.header.stamp = ros::Time::now();
}

void set_joint_trajectry_point_config(trajectory_msgs::JointTrajectoryPoint& joint_trajectory_point, const ros::Duration time_from_start){
	joint_trajectory_point.time_from_start = time_from_start; 
	joint_trajectory_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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

	// pose_follow_plannner3からの指令を待つ
	ROS_INFO_STREAM("Waiting message from pose_follow_plannner3");
	ros::topic::waitForMessage<geometry_msgs::Pose>("target_displacement");

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
	bool should_reset = false;
	while (ros::ok()) {
		// dtを計算
		double dt = (ros::Time::now() - t_last).toSec();
		t_last = ros::Time::now();
		kinematic_state.setVariableValues(joint_state_listener.data);
		ROS_INFO_STREAM("dt : " << dt);
		
		// Jacobianを取得
		const Eigen::MatrixXd jacobian = kinematic_state.getJacobian(joint_model_group_ptr); // TODO getJacobian()はlink_tまでのもの，それともtool_t?
		const Eigen::MatrixXd inv_jacobian = pseudo_inv(jacobian);

		// delta_qを計算
		const Eigen::VectorXd delta_q = inv_jacobian * target_displacement_listener.delta_x * 0.01;

		// JointTrajectoryPointに変換
		set_joint_trajectry_point_config(joint_trajectory_point, ros::Duration(ros::Time::now() - t_start));
		for (int i = 0; i < 7; i++) {
			joint_trajectory_point.positions.at(i) = joint_state_listener.data.position.at(i) + delta_q(i);
			if (should_reset) {
				joint_trajectory_point.positions = joint_state_listener.data.position;
				joint_trajectory_point.velocities.at(i) = 0;
			}
			else {
				joint_trajectory_point.velocities.at(i) = delta_q(i);
			}
		}

		//表示
		for (int i = 0; i < 7; i++) {
			std::cout << joint_trajectory_point.positions.at(i) << "\t";
		}
		std::cout << std::endl;
		for (int i = 0; i < 7; i++) {
			std::cout << joint_trajectory_point.velocities.at(i) << "\t";
		}
		std::cout << std::endl;

		// リミットを超えていたら，捨てる
		const auto max_joint_target_itr = std::max_element(joint_trajectory_point.positions.begin(), joint_trajectory_point.positions.end());
		const auto max_vel_target_itr = std::max_element(joint_trajectory_point.velocities.begin(), joint_trajectory_point.velocities.end());
		should_reset = false;
		if (std::abs(*max_joint_target_itr) > M_PI) {
			ROS_WARN_STREAM("Max joint limit over");
			should_reset = true;
		}
		else if (std::abs(*max_vel_target_itr) > M_PI) {
			ROS_WARN_STREAM("Max joint velocity limit over");
			should_reset = true;
		}
		else {
		 //JointTrajectoryをpublish
			set_joint_trajectry_config(joint_trajectory_msgs);
			joint_trajectory_msgs.points.at(0) = joint_trajectory_point;
			joint_trajectory_publisher.publish(joint_trajectory_msgs);
			ROS_INFO_STREAM("Publish once");
		}

		rate_timer.sleep();
		ros::spinOnce();
	}

	return 0;
}
