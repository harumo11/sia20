// このプログラムはHTC Viveのコントローラからの命令により
// ロボットアームの手先位置を計算し，それをpose_follow_downloaderに
// 流すプログラムです．

#include <algorithm>
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
	ros::Rate timer(50);

	// HTC Viveの設定
	const std::string vive_controller_id = "controller_LHR_066549FF";
	//const std::string vive_controller_id = "controller_LHR_3CCD7CA5";
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
	auto joint_value_command_publisher = node.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_value_command", 1);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point_msgs;
	
	int iter = 0;
	while (ros::ok()) {
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
			ROS_INFO_STREAM("Planning Success");
			//// planの一番最後の関節軸の情報を取り出す
			//const int plan_size = plan.trajectory_.joint_trajectory.points.size();
			//const int plan_size_mid = plan_size / 5;
			//ROS_WARN_STREAM("plan size mid : " << plan_size_mid);
			//joint_trajectory_point_msgs.positions =  plan.trajectory_.joint_trajectory.points.at(1).positions;
			joint_trajectory_point_msgs.positions =  plan.trajectory_.joint_trajectory.points.back().positions;
			//// 各関節角度のt と t-1の差分の最大値が10度以上ならプランニングをやり直す
			const double rad_10 = (60.0 / 180.0) * M_PI;
			std::vector<double> joint_diff;
			for (int i = 0; i < 6; i++) {	// joint_tは何度回ってもいいので無視する
				auto current_joint_value = move_group.getCurrentJointValues().at(i);
				auto target_joint_value  = joint_trajectory_point_msgs.positions.at(i);
				auto joint_diff_value = current_joint_value - target_joint_value;
				ROS_INFO_STREAM("current : " << current_joint_value * 180.0/M_PI << " target : " << target_joint_value * 180.0/M_PI<< " diff : " << joint_diff_value * 180.0/M_PI);
				joint_diff.push_back(std::abs(joint_diff_value));
			}
			const auto max_diff_itr = std::max_element(joint_diff.begin(), joint_diff.end());
			if (*max_diff_itr > rad_10) {	// １０度以上ならpublishしない
				ROS_WARN_STREAM("joint " << std::distance(joint_diff.begin(), max_diff_itr) << " is bigger than 20 deg : joint value is : " << *max_diff_itr * 180.0/M_PI);
			}
			else
			{
				// 各目標角度を1/10する
				//// 現在の関節角度を読み込む
				const std::vector<double> current_joint = move_group.getCurrentJointValues();
				//// target_joint - current_jointをしてjoint_displacementを計算する
				std::vector<double> joint_displacement;
				for (int i = 0; i < 7; i++) {
					joint_displacement.push_back(joint_trajectory_point_msgs.positions.at(i) - current_joint.at(i));
					//// target_joint = curren_joint + 0.1 * joint_displacementを計算する
					const double velocity_weight = 0.125;
					joint_trajectory_point_msgs.positions.at(i) = current_joint.at(i) + velocity_weight * joint_displacement.at(i);
				}
				
				// joint_trajectory_point_msgsをpublishする
				joint_value_command_publisher.publish(joint_trajectory_point_msgs);
				ROS_INFO_STREAM(joint_trajectory_point_msgs);
				ROS_INFO_STREAM("Target joints were published" << iter);
			}
		}
		else {
			ROS_WARN_STREAM("Planning Failed");
		}
		iter++;
		timer.sleep();
		ROS_INFO_STREAM("ONE CYCLE FINISHED");
	}
	
	return 0;
}
