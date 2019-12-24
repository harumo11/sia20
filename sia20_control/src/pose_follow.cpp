#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

	// HTC Viveの現在位置を読むための準備
	tf2_ros::Buffer tf_controller_buffer;
	tf2_ros::TransformListener tf_controller_listener(tf_controller_buffer);

	// HTC Viveの大きな丸ボタンが押された位置を{local_controller}フレームとして記録するための準備
	tf2_ros::StaticTransformBroadcaster static_controller_broadcaster;	// {world}からみた{controller}への変換行列をtfに流すためのクラス
	
	while (node_handler.ok()) {
		ROS_INFO_STREAM(vive_controller_listener.state);	// HTC Viveの現在のボタン状況を表示
		// コントローラのtriger押されたら{local_controller_frame}を作成する．
		// {local_controller_frame} : viveコントローラの初期位置の座標
		
		// コントローラの大きな丸ボタンが押されたら，現在の{world}からみたコントローラの座標を{local_controller}フレームとして作成(更新)する．
		if (vive_controller_listener.state.buttons.at(2) > 0) {	// HTC Viveの大きな丸ボタンが押されていた場合
			try {
				// 同時変換行列の取得を試みる
				geometry_msgs::TransformStamped transform_world_to_controller = tf_controller_buffer.lookupTransform("world", vive_id, ros::Time(0));
				ROS_INFO_STREAM("Transform\n" << transform_world_to_controller.transform.translation);
				
				// {world}からみた{controller}への変換行列をtfに配信する
				transform_world_to_controller.child_frame_id = "local_controller";
				static_controller_broadcaster.sendTransform(transform_world_to_controller);

			} catch (tf2::TransformException &ex) {	
				// 変換に失敗したらwhile文の先頭に戻る
				ROS_WARN_STREAM(ex.what());
				continue;	
			}
		}
		timer.sleep();
	}



	return 0;
}
