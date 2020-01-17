#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>

void count_down(const int count_sec = 5){
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
class JointTrajPointListener {
	public:
    trajectory_msgs::JointTrajectoryPoint data;
    void call_back(const trajectory_msgs::JointTrajectoryPoint msgs);
};

void JointTrajPointListener::call_back(const trajectory_msgs::JointTrajectoryPoint msgs){
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
std::string unstruct_joint_traj_point_msgs(const trajectory_msgs::JointTrajectoryPoint msgs){
	std::stringstream ss;
	ss << msgs.positions.at(0) << "," << msgs.positions.at(1) << "," << msgs.positions.at(2) << "," << msgs.positions.at(3) << "," << msgs.positions.at(4) << "," << msgs.positions.at(5) << "," << msgs.positions.at(6);
	return ss.str();
}

int main(int argc, char *argv[]) {

	
   // ROS初期化
   ros::init(argc, argv, "log_teacher_data_node");
   ros::NodeHandle node;
   ros::Rate timer(40);
 
   // リスナー宣言
   PoseListener dirt_pose_listener;
   PoseListener broom_pose_listener;
   PoseListener goal_pose_listener;
   JointStateListener joint_state_listener;
   JointTrajPointListener joint_traj_point_listener;
   ViveControllerListener vive_controller_listener;
   // サブスクライバー初期化
   ros::Subscriber dirt_pose_subscriber = node.subscribe("/ar_dirt_pose", 1, &PoseListener::call_back, &dirt_pose_listener);
   ros::Subscriber broom_pose_subscriber = node.subscribe("/ar_broom_pose", 1, &PoseListener::call_back, &broom_pose_listener);
   ros::Subscriber goal_pose_subscriber = node.subscribe("/ar_goal_pose", 1, &PoseListener::call_back, &goal_pose_listener);
   ros::Subscriber joint_state_subscriber = node.subscribe("/joint_states", 1, &JointStateListener::call_back, &joint_state_listener);
   ros::Subscriber joint_traj_point_subscriber = node.subscribe("/joint_value_command", 1, &JointTrajPointListener::call_back, &joint_traj_point_listener);
   ros::Subscriber vive_controller_subscriber = node.subscribe("/vive/controller_LHR_066549FF/joy", 1, &ViveControllerListener::call_back, &vive_controller_listener);
   // 全メッセージが到着するまで待つ
   ROS_INFO_STREAM("Waiting for dirt, goal, broom pose, joint state and vive controller topic");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_dirt_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_broom_pose");
   ros::topic::waitForMessage<geometry_msgs::Twist>("ar_goal_pose");
   ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
   ros::topic::waitForMessage<trajectory_msgs::JointTrajectoryPoint>("joint_value_command");
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
            std::ofstream log_file(file_path + "log_" + std::to_string(file_number++) + ".csv");
			std::stringstream ss;
			log_file << "dirt(x), dirt(y), dirt(z), dirt(r), dirt(p), dirt(w), goal(x), goal(y), goal(z), goal(r), goal(p), goal(w), broom(x), broom(y), broom(z), broom(r), broom(p), broom(w), state(s), state(l), state(e), state(u), state(r), state(b), state(t), command(s), command(l), command(e), command(u), command(r), command(b), command(t)" << std::endl;

			while (true) {
				log_file << unstruct_twist_msgs(dirt_pose_listener.data) << "," << unstruct_twist_msgs(goal_pose_listener.data) << "," << unstruct_twist_msgs(broom_pose_listener.data) << "," << unstruct_joint_state_msgs(joint_state_listener.data) << "," << unstruct_joint_traj_point_msgs(joint_traj_point_listener.data) << std::endl;
				ROS_DEBUG_STREAM_THROTTLE(10, "Recording NOW!");
				//ROS_INFO_STREAM(unstruct_twist_msgs(dirt_pose_listener.data) << "," << unstruct_twist_msgs(goal_pose_listener.data) << "," << unstruct_twist_msgs(broom_pose_listener.data) << "," << unstruct_joint_state_msgs(joint_state_listener.data) << "," << unstruct_joint_traj_point_msgs(joint_traj_point_listener.data));
				ros::spinOnce();
				timer.sleep();

				// Viveコントローラのボタンが押されたら記録ファイルを閉じてループを抜ける
				if (vive_controller_listener.data.buttons.at(3) == 1) {
					//Viveコントローラのボタンが誤検知を防ぐために５秒待つ
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
