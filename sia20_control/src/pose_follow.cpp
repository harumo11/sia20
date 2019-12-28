#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// HTC Viveのボタンの押し下げに関するトピックを検知するクラス
class ViveControllerListener {
	public:
		void call_back(const sensor_msgs::Joy msgs);
		sensor_msgs::Joy state;
};

void ViveControllerListener::call_back(const sensor_msgs::Joy msgs){
	this->state = msgs;
}

int main(int argc, char* argv[])
{
	const std::string vive_id = "controller_LHR_066549FF";
	const std::string vive_controller_topic_name = "/vive/" + vive_id + "/joy";
	

	// rosの設定
	ros::init(argc, argv, "pose_follow_node");
	ros::NodeHandle node_handler;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate timer(10);

	// HTC Viveから出ているJoyトピックを読むための準備
	ViveControllerListener vive_controller_listener;
	ros::Subscriber controller_subscriber = node_handler.subscribe(vive_controller_topic_name, 1, &ViveControllerListener::call_back, &vive_controller_listener);
	ROS_INFO_STREAM("Waiting HTC Vive controller topic");
	ros::topic::waitForMessage<sensor_msgs::Joy>(vive_controller_topic_name);

	// HTC Viveの現在位置をtfで読むための準備
	tf2_ros::Buffer tf_controller_buffer;
	tf2_ros::TransformListener tf_controller_listener(tf_controller_buffer);
	tf2_ros::TransformBroadcaster target_link_t_broadcaster; 

	// HTC Viveの大きな丸ボタンが押された位置を{local_controller}フレームとして記録するための準備
	tf2_ros::StaticTransformBroadcaster static_controller_broadcaster;	// {world}からみた{controller}への変換行列をtfに流すためのクラス

	// joint_trajectory用の時刻
	auto start_time = ros::Time::now();
	bool is_first_cycle = true;
	//ros::Publisher joint_streaming_publisher = node_handler.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
	
	// MoveItの設定
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");
	
	while (node_handler.ok()) {
		//ROS_INFO_STREAM(vive_controller_listener.state);	// HTC Viveの現在のボタン状況を表示
		// コントローラのtriger押されたら{local_controller_frame}を作成する．
		// {local_controller_frame} : viveコントローラの初期位置の座標
		
		// コントローラの大きな丸ボタンが押されたら，現在の{world}からみたコントローラの座標を{local_controller}フレームとして作成(更新)する．
		geometry_msgs::TransformStamped transform_world_to_controller;
		if (vive_controller_listener.state.buttons.at(2) > 0) {	// HTC Viveの大きな丸ボタンが押されていた場合
			try {
				// 同時変換行列の取得を試みる
				transform_world_to_controller = tf_controller_buffer.lookupTransform("world", vive_id, ros::Time(0));
				//ROS_INFO_STREAM("Transform\n" << transform_world_to_controller.transform.translation);
				
				// {world}からみた現在の{controller}を{local_controller}としてtfに配信する
				transform_world_to_controller.child_frame_id = "local_controller";
				static_controller_broadcaster.sendTransform(transform_world_to_controller);

			} catch (tf2::TransformException &ex) {	
				// 変換に失敗したらwhile文の先頭に戻る
				ROS_WARN_STREAM(ex.what());
				continue;	
			}
		}

		// {local_controller}から見た{controller}の同時変換行列を取得する．
		geometry_msgs::TransformStamped transform_local_controller_to_controller;
		try {
			// 同時変換行列の取得を試みる
			transform_local_controller_to_controller = tf_controller_buffer.lookupTransform("local_controller", vive_id, ros::Time(0));
			//ROS_INFO_STREAM(transform_local_controller_to_controller);
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}

		// {base_link}から見た{link_t}を取得する
		geometry_msgs::TransformStamped frame_link_t;
		try {
			geometry_msgs::TransformStamped frame_link_t = tf_controller_buffer.lookupTransform("base_link", "link_t", ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}

		// 手先目標位置を現在の手先位置とコントローラの変位から計算する
		auto target_link_t = transform_local_controller_to_controller;
		target_link_t.header.frame_id = "link_t";
		target_link_t.child_frame_id = "target_link_t";
		target_link_t.header.stamp = ros::Time::now();
		auto target_link_t_y = target_link_t.transform.translation.y;
		auto target_link_t_x = target_link_t.transform.translation.x;
		target_link_t.transform.translation.x = -1 * target_link_t_y;
		target_link_t.transform.translation.y = -1 * target_link_t_x;
		target_link_t.transform.translation.z = -1 * target_link_t.transform.translation.z;
		auto target_link_t_q_x = target_link_t.transform.rotation.x;
		auto target_link_t_q_y = target_link_t.transform.rotation.y;
		target_link_t.transform.rotation.x = -1 * target_link_t_q_y;
		target_link_t.transform.rotation.y = -1 * target_link_t_q_x;
		target_link_t.transform.rotation.z = -1 * target_link_t.transform.rotation.z;
		ROS_INFO_STREAM(target_link_t.transform.rotation);
		target_link_t_broadcaster.sendTransform(target_link_t);
		
		// {base_link}からみた{target_link_t}の値を取得する．
		geometry_msgs::TransformStamped transform_base_link_to_target_link_t;
		try {
			transform_base_link_to_target_link_t = tf_controller_buffer.lookupTransform("base_link", "target_link_t", ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}
		
		// {base_link}からみた{target_link_t}を目標としてプランニングを行う
		//// 目標座標をgeometry::TransformStampedをgeometry_msgs::Poseに変換
		geometry_msgs::Pose target_pose;
		target_pose.position.x = transform_base_link_to_target_link_t.transform.translation.x;
		target_pose.position.y = transform_base_link_to_target_link_t.transform.translation.y;
		target_pose.position.z = transform_base_link_to_target_link_t.transform.translation.z;
		target_pose.orientation = transform_base_link_to_target_link_t.transform.rotation;
		move_group.setPoseTarget(target_pose);
		// プランニングを行う
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
			ROS_INFO_STREAM("Planning Success");
			move_group.move();
		}
		else {
			ROS_WARN_STREAM("Planning Failed");
		}
		
		// 作成したプランから関節角度を取り出す
		//trajectory_msgs::JointTrajectoryPoint target_trajectory_point;
		//target_trajectory_point.positions = plan.trajectory_.joint_trajectory.points.at(9).positions; 	// 関節角度を取り出す
		//target_trajectory_point.velocities = plan.trajectory_.joint_trajectory.points.at(9).velocities; // 関節速度を取り出す
		////// time_from_startを設定
		//if (is_first_cycle)
		//{
		//	target_trajectory_point.time_from_start = ros::Duration(0.0);
		//}
		//else
		//{
		//	target_trajectory_point.time_from_start = ros::Time::now() - start_time;
		//}
		//trajectory_msgs::JointTrajectory target_trajectory;
		//target_trajectory.header.stamp = ros::Time::now();
		//target_trajectory.joint_names = {"joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
		//target_trajectory.points.push_back(target_trajectory_point);
		////joint_streaming_publisher.publish(target_trajectory);
		//ROS_INFO_STREAM("joint_path_command is published");

		timer.sleep();
	}

	return 0;
}
