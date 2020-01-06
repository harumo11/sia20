#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>

// HTC Viveのボタンに関する情報を購読するクラス
class ViveController {
	public:
		void call_back(const sensor_msgs::Joy msgs);
		sensor_msgs::Joy state;
};

void ViveController::call_back(const sensor_msgs::Joy msgs){
	this->state = msgs;
}


int main(int argc, char* argv[])
{
	// rosの設定
	ros::init(argc, argv, "pose_follow_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate timer(1);

	// HTC Viveの設定
	const std::string vive_controller_id = "controller_LHR_066549FF";
	const std::string vive_controller_topic_name = "/vive/" + vive_controller_id + "/joy";
	ViveController vive_controller;
	auto vive_controller_subscriber = node.subscribe(vive_controller_topic_name, 1, &ViveController::call_back, &vive_controller);
	ROS_INFO_STREAM("Waiting HTC Vive controller topic");
	ros::topic::waitForMessage<sensor_msgs::Joy>(vive_controller_topic_name);

	// tf2の設定
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	tf2_ros::StaticTransformBroadcaster tf_static_broadcaster;
	tf2_ros::TransformBroadcaster tf_broadcaster;

	// moveitの設定
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");

	// joint_trajectoryの設定
	auto joint_streaming_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/joint_command", 1);
	const auto start_time = ros::Time::now();
	bool is_first_cycle = true;
	//// 現在位置の送信
	const auto current_joint_values = move_group.getCurrentJointValues();
	trajectory_msgs::JointTrajectory trajectory_msgs;
	trajectory_msgs::JointTrajectoryPoint trajectory_point_msgs;
	////// trajectory_pointの設定
	for (auto&& joint : current_joint_values){
		trajectory_point_msgs.positions.push_back(joint);
		trajectory_point_msgs.velocities.push_back(0);
	}
	trajectory_point_msgs.time_from_start = ros::Time::now() - start_time;
	////// trajectory_msgsの設定
	trajectory_msgs.header.stamp = ros::Time::now();
	trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
	trajectory_msgs.points.push_back(trajectory_point_msgs);
	joint_streaming_publisher.publish(trajectory_msgs);
	ROS_INFO_STREAM("Initial target joints were published");
	
	int iter = 0;
	while (node.ok()) {
		if (vive_controller.state.buttons.at(2) > 0) {	// もし大きな丸いボタンが押されたら
			ROS_INFO_STREAM("Vive controller's big circle button is pushed");

			// sia20dをenableにする
			std_srvs::Trigger robot_enable;
			ros::service::call("/robot_enable", robot_enable);

			// {world}から見た{controller}より，staticなフレームである{local_controller}を生成
			geometry_msgs::TransformStamped tf_local_controller_frame;
			try {
				tf_local_controller_frame = tf_buffer.lookupTransform("world", vive_controller_id, ros::Time(0));
			} catch (tf2::TransformException& ex) {
				ROS_WARN_STREAM(ex.what());
				continue;
			}
			
			// {base_link}から見た{link_t}より，staticなフレームである{local_link_t}を生成
			geometry_msgs::TransformStamped tf_local_link_t_frame;
			try {
				tf_local_link_t_frame = tf_buffer.lookupTransform("link_t", "base_link", ros::Time(0));
			} catch (tf2::TransformException& ex) {
				ROS_WARN_STREAM(ex.what());
				continue;
			}

			// {local_controller}をtfに登録
			tf_local_controller_frame.child_frame_id = "local_controller";
			tf_static_broadcaster.sendTransform(tf_local_controller_frame);

			// {local_link_t}をtfに登録
			tf_local_link_t_frame.header.frame_id = "local_link_t";
			tf_local_link_t_frame.child_frame_id = "base_link";
			tf_static_broadcaster.sendTransform(tf_local_link_t_frame);

			// {local_controller}と{local_link_t}の関係を同一のフレームとすることをtfに登録
			geometry_msgs::TransformStamped tf_local_controller_frame_to_local_link_t;
			tf_local_controller_frame_to_local_link_t.header.frame_id = "local_controller";	// "local_controller"を親フレームとして設定
			tf_local_controller_frame_to_local_link_t.child_frame_id  = "local_link_t";		// "local_link_t"を子フレームとして登録
			tf_local_controller_frame_to_local_link_t.header.stamp = ros::Time::now();
			tf_local_controller_frame_to_local_link_t.transform.translation.x = 0; 
			tf_local_controller_frame_to_local_link_t.transform.translation.y = 0; 
			tf_local_controller_frame_to_local_link_t.transform.translation.z = 0; 
			tf_local_controller_frame_to_local_link_t.transform.rotation.x = 0;
			tf_local_controller_frame_to_local_link_t.transform.rotation.y = 0;
			tf_local_controller_frame_to_local_link_t.transform.rotation.z = 0;
			tf_local_controller_frame_to_local_link_t.transform.rotation.w = 1;
			tf_static_broadcaster.sendTransform(tf_local_controller_frame_to_local_link_t);
		}

		// {base_link}から見た{controller}を目標としてプランニングを行う
		//// {controller}をgeometory::TransformStampedからgeometory_msgs::Poseに型変換
		geometry_msgs::Pose target_link_t_pose;
		geometry_msgs::TransformStamped tf_base_link_vive_controller;

		//// {base_link}から見た{controller}の座標変換を計算
		try {
			tf_base_link_vive_controller = tf_buffer.lookupTransform("base_link", vive_controller_id, ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}
		target_link_t_pose.position.x = tf_base_link_vive_controller.transform.translation.x;
		target_link_t_pose.position.y = tf_base_link_vive_controller.transform.translation.y;
		target_link_t_pose.position.z = tf_base_link_vive_controller.transform.translation.z;
		target_link_t_pose.orientation = tf_base_link_vive_controller.transform.rotation;
		ROS_INFO_STREAM(target_link_t_pose);

		//// planを作成
		move_group.setPoseTarget(target_link_t_pose);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
			move_group.move();
			//ROS_INFO_STREAM("Planning Success");
			////// planの一番最後の関節軸の情報を取り出す
			//const int plan_point_size = plan.trajectory_.joint_trajectory.points.size();
			//trajectory_point_msgs.positions =  plan.trajectory_.joint_trajectory.points.back().positions;
			//trajectory_point_msgs.velocities = plan.trajectory_.joint_trajectory.points.back().velocities;
			////trajectory_point_msgs.velocities = {0,0,0,0,0,0,0};
			////trajectory_point_msgs.velocities = plan.trajectory_.joint_trajectory.points.at(1).velocities;
			//trajectory_point_msgs.time_from_start = ros::Time::now() - start_time;
			//trajectory_msgs.header.stamp = ros::Time::now();
			//trajectory_msgs.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
			//trajectory_msgs.points.at(0) = trajectory_point_msgs;
			//joint_streaming_publisher.publish(trajectory_msgs);
			//ROS_INFO_STREAM("Target joints were published" << iter);
		}
		else {
			ROS_WARN_STREAM("Planning Failed");
			continue;
		}
		iter++;

		ROS_INFO_STREAM("ONE CYCLE FINISHED");
		timer.sleep();
	}

	
	return 0;
}
