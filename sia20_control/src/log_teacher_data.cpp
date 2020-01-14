#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>

bool should_init = false;

void call_back(sensor_msgs::Joy msgs) {
	if (msgs.buttons[3] == 1) { // viveコントローラの持ち手にある楕円形のボタンのどちらかが押されたら
		ROS_INFO_STREAM("Move init pose");
		should_init = true;
	}
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
   ros::Subscriber dirt_pose_subscriber = node.subscribe(
       "/ar_dirt_pose", 1, &PoseListener::call_back, &dirt_pose_listener);
   ros::Subscriber broom_pose_subscriber = node.subscribe(
       "/ar_broom_pose", 1, &PoseListener::call_back, &broom_pose_listener);
   ros::Subscriber goal_pose_subscriber = node.subscribe(
       "/ar_goal_pose", 1, &PoseListener::call_back, &goal_pose_listener);
   // 全メッセージが到着するまで待つ
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_dirt_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_broom_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_goal_pose");
 
   // MoveIt初期化
   ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
   ros::Subscriber subs =
       node.subscribe("/vive/controller_LHR_066549FF/joy", 1, call_back);
   static const std::string PLANNING_GROUP = "manipulator";
   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   const std::vector<double> init_joint_values = {
       1.61967921257,   -0.316306859255, -1.86695694923,   -1.14934372902,
       -0.374367237091, -0.273443460464, 0.000240624460275};
 
   ROS_INFO_STREAM("Logging node starts");
 
   ros::Rate timer(50);
	
   int iter = 0;
   while (ros::ok()) {
	   //もし，初期位置ポーズに戻るボタンが押されたら
	   if (should_init == true) {
         move_group.setJointValueTarget(init_joint_values);
         move_group.move();
         should_init = false;
         ros::Duration(2.0).sleep();
       }
	   else {
		   const std::string log_name = "log_" + std::to_string(iter++) + ".csv";
		   std::ofstream file(log_name);;

		   // 各arマーカの座標を格納
		   std::stringstream log_contents;
		   log_contents << unstruct_twist_msgs(dirt_pose_listener.data) << unstruct_twist_msgs(broom_pose_listener.data) << unstruct_twist_msgs(goal_pose_listener.data) << std::endl;
		   // 関節軸の値を格納(s,l,e,u,r,b,tの順)
		   std::vector<double> joints = move_group.getCurrentJointValues();
		   for (auto&& joint : joints){
			   log_contents << ",\t" << std::to_string(joint);
		   }
		   file << log_contents.str();	// マーカ座標と軸の角度を保存
		   file.close();
	   }
	   
       timer.sleep();
   }
 
   return 0;
}
