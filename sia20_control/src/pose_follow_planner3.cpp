// このプログラムはHTC Viveのコントローラからの命令により
// 現在の手先位置と目標位置の差分を計算し，それをpose_follow_executer2に流すプログラムです．

#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h> 
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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

class JointStateListener {
	public:
		sensor_msgs::JointState data;
		void call_back(const sensor_msgs::JointState msgs);
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->data = msgs;
}


int main(int argc, char* argv[])
{
	// rosの設定
	ros::init(argc, argv, "pose_follow_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	//ros::Rate timer(100);
	ros::Rate timer(40);

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
	auto target_displacement_publisher = node.advertise<geometry_msgs::Pose>("/target_displacement", 1);
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_point_msgs;
	// target_poseの設定
	auto target_pose_publisher = node.advertise<geometry_msgs::Pose>("target_pose", 1);
	// pose_following/cmd_vel用のpublisher
	auto target_pose_twist_publisher = node.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);
	
	int iter = 0;
	while (ros::ok()) {
		if (vive_controller.state.buttons.at(2) > 0) {	// もし大きな丸いボタンが押されたら
			ROS_INFO_STREAM("Vive controller's big circle button is pushed");

			// {world}から見た{vive_controller}より，staticなフレームである{local_controller}を生成
			geometry_msgs::TransformStamped tf_local_controller_frame;
			try {
				tf_local_controller_frame = tf_buffer.lookupTransform("world", vive_controller_id, ros::Time(0));
			} catch (tf2::TransformException& ex) {
				ROS_WARN_STREAM(ex.what());
				continue;
			}
			// {local_controller}をtfに登録
			tf_local_controller_frame.child_frame_id = "local_controller";
			tf_local_controller_frame.header.frame_id = "world";
			tf_static_broadcaster.sendTransform(tf_local_controller_frame);

			// {base_link}から見た{link_t}より，staticなフレームである{local_link_t}を生成
			geometry_msgs::TransformStamped tf_local_link_t;
			try {
				tf_local_link_t = tf_buffer.lookupTransform("base_link", "link_t", ros::Time(0));
			} catch (tf2::TransformException& ex) {
				ROS_WARN_STREAM(ex.what());
				continue;
			}
			// {local_link_t}をtfに登録
			tf_local_link_t.child_frame_id = "local_link_t";
			tf_local_link_t.header.frame_id = "base_link";
			tf_static_broadcaster.sendTransform(tf_local_link_t);
		}

		// {local_controller}から見た{vive_controller}の座標変換を計算
		geometry_msgs::TransformStamped tf_local_controller_vive_controller;
		try {
			tf_local_controller_vive_controller = tf_buffer.lookupTransform("local_controller", vive_controller_id, ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}

		// {local_link_t}にtf_local_controller_vive_controllerを加え,{target_link_t}を作る
		geometry_msgs::TransformStamped target_link_t;
		// {base_link}から見た{local_link_t}を取得
		geometry_msgs::TransformStamped tf_base_link_local_link_t;
		try {
			tf_base_link_local_link_t = tf_buffer.lookupTransform("base_link", "local_link_t", ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}
		// 位置を足し算
		target_link_t.transform.translation.x = tf_base_link_local_link_t.transform.translation.x + tf_local_controller_vive_controller.transform.translation.x;
		target_link_t.transform.translation.y = tf_base_link_local_link_t.transform.translation.y + tf_local_controller_vive_controller.transform.translation.y;
		target_link_t.transform.translation.z = tf_base_link_local_link_t.transform.translation.z + tf_local_controller_vive_controller.transform.translation.z;
		// 姿勢を足し算
		tf2::Quaternion q_new, q_org, q_rot;
		tf2::convert(tf_base_link_local_link_t.transform.rotation, q_org);
		//ROS_INFO_STREAM("tf_base_link_local_link_t quat" << tf_base_link_local_link_t.transform.rotation);
		//ROS_INFO_STREAM("local_link_t(q_org) " << q_org.x() << ", " << q_org.y() << ", " << q_org.z() << ", " << q_org.w());
		tf2::convert(tf_local_controller_vive_controller.transform.rotation, q_rot);
		//ROS_INFO_STREAM("local_controller(q_rot) " << q_rot.x() << ", " << q_rot.y() << ", " << q_rot.z() << ", " << q_rot.w());
		q_new = q_rot * q_org;
		q_new.normalize();
		//ROS_INFO_STREAM("q_new " << q_new.x() << ", " << q_new.y() << ", " << q_new.z() << ", " << q_new.w());
		//tf2::convert(q_new, target_link_t.transform.rotation);
		target_link_t.transform.rotation = tf2::toMsg(q_new);
		// 親フレーム，子フレームを設定
		target_link_t.header.frame_id = "base_link";
		target_link_t.child_frame_id  = "target_link_t";
		tf_broadcaster.sendTransform(target_link_t);
		
		// {base_link}から見た{target_link_t}を取得する
		geometry_msgs::TransformStamped tf_base_link_target_link_t;
		try {
			tf_base_link_target_link_t = tf_buffer.lookupTransform("base_link", "target_link_t", ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}

		// tf_base_link_target_link_tをgeometry_msgs::Poseに型変換
		geometry_msgs::Pose target_link_t_pose;
		target_link_t_pose.position.x = tf_base_link_target_link_t.transform.translation.x;
		target_link_t_pose.position.y = tf_base_link_target_link_t.transform.translation.y;
		target_link_t_pose.position.z = tf_base_link_target_link_t.transform.translation.z;
		target_link_t_pose.orientation = tf_base_link_target_link_t.transform.rotation;

		// 現在の手先座標を得る
		//geometry_msgs::PoseStamped current_link_t_pose = move_group.getCurrentPose();
		geometry_msgs::TransformStamped tf_base_link_link_t;
		try {
			tf_base_link_link_t = tf_buffer.lookupTransform("base_link", "link_t", ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_WARN_STREAM(ex.what());
			continue;
		}
		geometry_msgs::PoseStamped current_link_t_pose;
		current_link_t_pose.pose.position.x = tf_base_link_link_t.transform.translation.x;
		current_link_t_pose.pose.position.y = tf_base_link_link_t.transform.translation.y;
		current_link_t_pose.pose.position.z = tf_base_link_link_t.transform.translation.z;
		current_link_t_pose.pose.orientation = tf_base_link_link_t.transform.rotation;

		// 目標手先座標(target_link_t)と現在手先座標(current_link_t_pose)の差分を計算する
		// 位置の差分
		geometry_msgs::Pose displacement_link_t;
		displacement_link_t.position.x = target_link_t_pose.position.x - current_link_t_pose.pose.position.x;
		displacement_link_t.position.y = target_link_t_pose.position.y - current_link_t_pose.pose.position.y;
		displacement_link_t.position.z = target_link_t_pose.position.z - current_link_t_pose.pose.position.z;
		// 姿勢の差分
		tf2::Quaternion target_link_t_quat(target_link_t_pose.orientation.x, target_link_t_pose.orientation.y, target_link_t_pose.orientation.z, target_link_t_pose.orientation.w);
		tf2::Quaternion current_link_t_quat(current_link_t_pose.pose.orientation.x, current_link_t_pose.pose.orientation.y, current_link_t_pose.pose.orientation.z, current_link_t_pose.pose.orientation.w);
		tf2::Quaternion displacement_link_t_quat = target_link_t_quat * current_link_t_quat.inverse();
		displacement_link_t_quat.normalize();
		displacement_link_t.orientation.x = displacement_link_t_quat.x();
		displacement_link_t.orientation.y = displacement_link_t_quat.y();
		displacement_link_t.orientation.z = displacement_link_t_quat.z();
		displacement_link_t.orientation.w = displacement_link_t_quat.w();
		//表示
		double r,p,w;
		double t_r, t_p, t_w;
		double c_r, c_p, c_w;
		tf2::Matrix3x3(displacement_link_t_quat).getRPY(r,p,w);
		tf2::Matrix3x3(target_link_t_quat).getRPY(t_r, t_p, t_w);
		tf2::Matrix3x3(current_link_t_quat).getRPY(c_r, c_p, c_w);

		ROS_INFO_STREAM("relative\t"       << r   << "\t" << p   << "\t" << w);
		ROS_INFO_STREAM("current link_t\t" << c_r << "\t" << c_p << "\t" << c_w);
		ROS_INFO_STREAM("target  link_t\t" << t_r << "\t" << t_p << "\t" << t_w);
		
		ROS_INFO_STREAM(displacement_link_t);

		target_displacement_publisher.publish(displacement_link_t);

		// pose_following/cmd_vel
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = displacement_link_t.position.x;
		cmd_vel.linear.y = displacement_link_t.position.y;
		cmd_vel.linear.z = displacement_link_t.position.z;
		cmd_vel.angular.x = r;
		cmd_vel.angular.y = p;
		cmd_vel.angular.z = w;
		ROS_INFO_STREAM("/cmd_vel\t" << cmd_vel);
		target_pose_twist_publisher.publish(cmd_vel);

		iter++;
		timer.sleep();
		ROS_INFO_STREAM("ONE CYCLE FINISHED");
	}
	
	return 0;
}
