#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>

class ViveListener {
	public:
		void call_back(sensor_msgs::Joy msgs);
		bool should_init();
		bool button_pressed = false;
};

void ViveListener::call_back(sensor_msgs::Joy msgs) {
	if (msgs.buttons[3] == 1) { // viveコントローラの持ち手にある楕円形のボタンのどちらかが押されたら
		ROS_INFO_STREAM("Move init pose");
		this->button_pressed = true;
	}
}

/**
 * @brief Viveコントローラのボタンが押されたかどうかを保持する関数
 * もし押されていたらtrueが返る．押されていなかったらfalseが返る．
 * この関数が呼ばれた後，メンバ変数のbutton_pressedはfalseになる．
 *
 * @return もし押されていたらtrueが返る．押されていなかったらfalseが返る
 */
bool ViveListener::should_init(){
	auto tmp_button = this->button_pressed;
	this->button_pressed = false;
	return tmp_button;
}

class PoseListener {
	public:
		geometry_msgs::Twist data;
		void call_back(geometry_msgs::Twist msgs);
};

void PoseListener::call_back(geometry_msgs::Twist msgs) { this->data = msgs; }

std::string unstruct_twist_msgs(const geometry_msgs::Twist msgs){
	std::stringstream ss;
	ss << msgs.linear.x << ",\t" << msgs.linear.y << ",\t" << msgs.linear.z << ",\t" << msgs.angular.x << ",\t" << msgs.angular.y << ",\t" << msgs.angular.z;
	return ss.str();
}

int main(int argc, char *argv[]) {

	
   // ROS初期化
   ros::init(argc, argv, "move_init_pose_srv_node");
   ros::NodeHandle node;
   ros::AsyncSpinner spinner(2);
   spinner.start();
 
   // サブスクライバー初期化
   PoseListener dirt_pose_listener;
   PoseListener broom_pose_listener;
   PoseListener goal_pose_listener;
   ViveListener vive_listener;
   ros::Subscriber dirt_pose_subscriber = node.subscribe(
       "/ar_dirt_pose", 1, &PoseListener::call_back, &dirt_pose_listener);
   ros::Subscriber broom_pose_subscriber = node.subscribe(
       "/ar_broom_pose", 1, &PoseListener::call_back, &broom_pose_listener);
   ros::Subscriber goal_pose_subscriber = node.subscribe(
       "/ar_goal_pose", 1, &PoseListener::call_back, &goal_pose_listener);
   ros::Subscriber subs =
       node.subscribe("/vive/controller_LHR_066549FF/joy", 1, &ViveListener::call_back, &vive_listener);
   // 全メッセージが到着するまで待つ
   ROS_INFO_STREAM("Waiting for dirt, goal, broom pose, joint state and vive controller topic");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_dirt_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_broom_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_goal_pose");
   ros::topic::waitForMessage<sensor_msgs::Joy>("/vive/controller_LHR_066549FF/joy");
   ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
 
   // MoveIt初期化
   static const std::string PLANNING_GROUP = "manipulator";
   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   const std::vector<double> init_joint_values = {
       1.61967921257,   -0.316306859255, -1.86695694923,   -1.14934372902,
       -0.374367237091, -0.273443460464, 0.000240624460275};
 
   ROS_INFO_STREAM("Logging node starts");
 
   ros::Rate timer(50);
	
   const int logged_file_number = 20;
   for (int file_number = 0; file_number < logged_file_number; file_number++) {
	   //記録ファイル作成
	   std::ofstream log_file("log_" + std::to_string(file_number) + ".csv");	

	   while (ros::ok()) {
		   //もしコントローラのボタンが押されたら
		   if (vive_listener.should_init() == true) {
			   //記録ファイルを閉じる
			   log_file.close();
			   //while文を抜け出す
			   break;
		   }
		   else {//コントローラのボタンが押されていないなら
			   //サブスクライブしたデータを記録ファイルに追加
			   std::stringstream log_contents;
		   	   log_contents << unstruct_twist_msgs(dirt_pose_listener.data) << unstruct_twist_msgs(broom_pose_listener.data) << unstruct_twist_msgs(goal_pose_listener.data) << std::endl;
		   	   // 関節軸の値を格納(s,l,e,u,r,b,tの順)
		   	   std::vector<double> joints = move_group.getCurrentJointValues();
		   	   for (auto&& joint : joints){
		   	       log_contents << ",\t" << std::to_string(joint);
		   	   }
				   log_file << log_contents.str();	// マーカ座標と軸の角度を保存
		  }
		  //スリープ(50Hz)
		  timer.sleep();
	   }
   }

   return 0;
}
