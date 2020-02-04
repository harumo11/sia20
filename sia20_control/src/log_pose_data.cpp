//このプログラムはロボットの手先の追従性を確認するものです．
//Viveコントローラからの命令と，実際のロボットの手先の位置を取得します．
//
//使い方はコントローラの大きなマルボタンを押すと位置関係の初期化＆追従開始
//コントローラの横の楕円のボタンを押すとコントローラの位置とロボットの手先座標
//の記録を開始します．

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>

void count_down(const int count_sec = 3){
	for (int i = count_sec; i > 0; i--) {
		ROS_INFO_STREAM("Count down : " << i);
		ros::Duration(1.0).sleep();
	}
	ROS_INFO_STREAM("Rec starts!");
}

// Viveコントローラのボタンをサブスクライブするクラス
class ViveControllerListener {
    public:
    sensor_msgs::Joy data;
    void call_back(const sensor_msgs::Joy msgs);
};

void ViveControllerListener::call_back(const sensor_msgs::Joy msgs){ 
	this->data = msgs; 
}

// 目標関節軸をサブスクライブするクラス
class JointTrajectoryListener {
	public:
    trajectory_msgs::JointTrajectory data;
    void call_back(const trajectory_msgs::JointTrajectory msgs);
};

void JointTrajectoryListener::call_back(const trajectory_msgs::JointTrajectory msgs){
	this->data = msgs;  
}

// 現在の関節角度をサブスクライブするクラス
class JointStateListener {
	public:
    sensor_msgs::JointState data;
    void call_back(const sensor_msgs::JointState msgs);
};

void JointStateListener::call_back(const sensor_msgs::JointState msgs){
	this->data = msgs;  
}

// arマーカから獲得できる座標をサブスクライブするクラス
class PoseListener {
	public:
		geometry_msgs::Twist data;
		void call_back(const geometry_msgs::Twist msgs);
};

void PoseListener::call_back(const geometry_msgs::Twist msgs) {
	this->data = msgs; 
}

// 届いたposeメッセージのフォーマットを記録用に変更する関数
std::string unstruct_twist_msgs(const geometry_msgs::Twist msgs){
	std::stringstream ss;
	ss << msgs.linear.x << "," << msgs.linear.y << "," << msgs.linear.z << "," << msgs.angular.x << "," << msgs.angular.y << "," << msgs.angular.z;
	return ss.str();
}

// 届いたJointStateメッセージのフォーマットを記録用に変更する関数
std::string unstruct_joint_state_msgs(const sensor_msgs::JointState msgs){
	std::stringstream ss;
	ss << msgs.position.at(0) << "," << msgs.position.at(1) << "," << msgs.position.at(2) << "," << msgs.position.at(3) << "," << msgs.position.at(4) << "," << msgs.position.at(5) << "," << msgs.position.at(6);
	return ss.str();
}

// 届いたJointTrajectoryメッセージのフォーマットを記録用に変更する関数
std::string unstruct_joint_trajectory_msgs(const trajectory_msgs::JointTrajectory msgs){
	std::stringstream ss;
	ROS_INFO_STREAM("unstruct joint trajectory msgs");
	ss << msgs.points.at(0).positions.at(0) << "," << msgs.points.at(0).positions.at(1) << "," << msgs.points.at(0).positions.at(2) << "," << msgs.points.at(0).positions.at(3) << "," << msgs.points.at(0).positions.at(4) << "," << msgs.points.at(0).positions.at(5) << "," << msgs.points.at(0).positions.at(6);
	msgs.points.at(0).positions.at(0);
	return ss.str();
}

// PoseStampedに含まれるQuaternionをRPYを含むTwistに変換する関数
geometry_msgs::Twist pose2twist(const geometry_msgs::PoseStamped pose){
	// Quaternionをrpyに変換
	tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	double r, p, w;
	tf2::Matrix3x3(quat).getRPY(r, p, w);

	// Twistに格納
	geometry_msgs::Twist twist;
	twist.linear.x = pose.pose.position.x;
	twist.linear.y = pose.pose.position.y;
	twist.linear.z = pose.pose.position.z;
	twist.angular.x = r;
	twist.angular.y = p;
	twist.angular.z = w;

	return twist;
}


int main(int argc, char *argv[]) {

	
   // ROS初期化
   ros::init(argc, argv, "log_teacher_data_node");
   ros::NodeHandle node;
   ros::AsyncSpinner spinner(2);
   spinner.start();
   ros::Rate timer(40);
 
   // リスナー宣言
   ViveControllerListener vive_controller_listener;

   // サブスクライバー初期化
   ros::Subscriber vive_controller_subscriber = node.subscribe("/vive/controller_LHR_066549FF/joy", 1, &ViveControllerListener::call_back, &vive_controller_listener);

   // moveit初期化
   moveit::planning_interface::MoveGroupInterface move_group("manipulator");

   //tf初期化
   tf2_ros::Buffer tf_buffer;
   tf2_ros::TransformListener tf_listener(tf_buffer);

   // 全メッセージが到着するまで待つ
   ROS_INFO_STREAM("Waiting for dirt, goal, broom pose, joint state and vive controller topic");
   ros::topic::waitForMessage<sensor_msgs::Joy>("/vive/controller_LHR_066549FF/joy");
 
    ROS_INFO_STREAM("Logging node starts");
    int file_number = 0;
	
    while (ros::ok())
    {
        ROS_INFO_STREAM("Press Side button of Vive controller to start rec");
        ros::spinOnce();
		timer.sleep();
        
        // viveコントローラのボタンが押されたらレコーディングスタート
        if (vive_controller_listener.data.buttons.at(3) == 1){
            ROS_INFO_STREAM("Side button of Vive controller is pressed");
			count_down();
            
            // 記録用ファイル作成
			ros::spinOnce();
            const std::string file_path = "/home/robot/catkin_ws/src/sia20/sia20_control/log/";
            std::ofstream log_file(file_path + "log_pose" + std::to_string(file_number++) + ".csv");
			std::stringstream ss;
			log_file << "hand pose (x)" << "," << "hand pose (y)" << "," << "hand pose (z)" << "," << "hand pose (r)" << "," << "hand pose (p)" << "," << "hand pose (w)" << "controller pose (x)" << "," << "controller pose (y)" << "," << "controller pose (z)" << "," << "controller pose (r)" << "," << "controller pose (p)" << "," << "controller pose (w)" << std::endl;

			while (true) {
				// moveitから現在の手先位置を取得
				const auto current_hand_msgs = pose2twist(move_group.getCurrentPose());
				// vive controllerの現在の位置を取得
				//TODO ここからやって
				log_file << unstruct_twist_msgs(current_hand_msgs) << "," <<  std::endl;
				ROS_DEBUG_STREAM_THROTTLE(10, "Recording NOW!");
				//ROS_INFO_STREAM(unstruct_twist_msgs(dirt_pose_listener.data) << "," << unstruct_twist_msgs(goal_pose_listener.data) << "," << unstruct_twist_msgs(broom_pose_listener.data) << "," << unstruct_joint_state_msgs(joint_state_listener.data) << "," << unstruct_joint_traj_point_msgs(joint_traj_point_listener.data));
				ros::spinOnce();
				timer.sleep();

				// Viveコントローラのボタンが押されたら記録ファイルを閉じてループを抜ける
				if (vive_controller_listener.data.buttons.at(3) == 1) {
					//Viveコントローラのボタンが誤検知を防ぐために５秒待つ
					ROS_INFO_STREAM("Button at Vive controller is pushed. Wait 5sec");
					ros::Duration(5.0).sleep();
					ROS_ERROR_STREAM("Rec fin!");
					//ファイルを閉じる
					log_file.close();
					//ループを抜ける
					break;
				}
			}
        }
    }
   return 0;
}
