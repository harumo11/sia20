#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
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
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate timer(50);

	// HTC Viveから出ているJoyトピックを読むための準備
	ViveControllerListener vive_controller_listener;
	ros::Subscriber controller_subscriber = node_handler.subscribe(vive_controller_topic_name, 1, &ViveControllerListener::call_back, &vive_controller_listener);
	ROS_INFO_STREAM("Waiting HTC Vive controller topic");
	ros::topic::waitForMessage<sensor_msgs::Joy>(vive_controller_topic_name);

	// HTC Viveの現在位置をtfで読むための準備
	tf2_ros::Buffer tf_controller_buffer;
	tf2_ros::TransformListener tf_controller_listener(tf_controller_buffer);

	// HTC Viveの大きな丸ボタンが押された位置を{local_controller}フレームとして記録するための準備
	tf2_ros::StaticTransformBroadcaster static_controller_broadcaster;	// {world}からみた{controller}への変換行列をtfに流すためのクラス
	
	while (node_handler.ok()) {
		ROS_INFO_STREAM(vive_controller_listener.state);	// HTC Viveの現在のボタン状況を表示
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
		ROS_INFO_STREAM(transform_local_controller_to_controller);
		geometry_msgs::TransformStamped target_link_t;
		// 位置
		target_link_t.transform.translation.x = frame_link_t.transform.translation.x + transform_local_controller_to_controller.transform.translation.x;
		target_link_t.transform.translation.y = frame_link_t.transform.translation.y + transform_local_controller_to_controller.transform.translation.y;
		target_link_t.transform.translation.z = frame_link_t.transform.translation.z + transform_local_controller_to_controller.transform.translation.z;
		// 姿勢
		tf2::Quaternion frame_link_t_quat, transform_local_controller_to_controller_quat;
		tf2::fromMsg(frame_link_t.transform.rotation, frame_link_t_quat);	// 計算のためgeometry_msgsをtfの型に変換
		tf2::fromMsg(transform_local_controller_to_controller.transform.rotation, transform_local_controller_to_controller_quat); // 計算のためgeometry_msgsをtfの型に変換
		tf2::Quaternion target_link_t_quat = transform_local_controller_to_controller_quat * frame_link_t_quat;	// 回転を計算する．rotation * originの順で掛け合わせる
		target_link_t_quat.normalize();
		target_link_t.transform.rotation = tf2::toMsg(target_link_t_quat);	// tfの型からgeometry_msgsの型に再変換

		// {link_base}からみた{link_t_target}をtfに送信
		tf2_ros::TransformBroadcaster target_link_t_broadcaster; 
		target_link_t.header.stamp = ros::Time::now();
		target_link_t.header.frame_id = "base_link";
		target_link_t.child_frame_id  = "link_t_target";
		target_link_t_broadcaster.sendTransform(target_link_t);
		ROS_INFO_STREAM(target_link_t);
		
		timer.sleep();
	}



	return 0;
}
