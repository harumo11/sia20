//このプログラムはdynetを使用して学習した箒の使用方法を実際に推測し，実行するための
//プログラムです．学習済みモデルを読み込み，ニューラルネットを構築し
//そこにリアルタイムで得たセンサーからの情報を入力として流し込み
//出力として手先の速度を得ます．その後，ROSでその速度をpublishします．

#include <iostream>
#include <algorithm>
#include <fstream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <fanda/Csv.hpp>
#include <dynet/io.h>
#include <dynet/training.h>
#include <dynet/expr.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class PoseListener {
	public:
		geometry_msgs::Twist data;
		void call_back(const geometry_msgs::Twist msgs){ this->data = msgs; }
};

std::vector<double> twist_to_vector(const geometry_msgs::Twist msgs){
	std::vector<double> vec;

	//位置を挿入
	vec.push_back(msgs.linear.x);
	vec.push_back(msgs.linear.y);
	vec.push_back(msgs.linear.z);
	//姿勢を挿入
	vec.push_back(msgs.angular.x);
	vec.push_back(msgs.angular.y);
	vec.push_back(msgs.angular.z);

	return vec;
}

geometry_msgs::Twist pose_to_twist(const geometry_msgs::PoseStamped pose){
	//Quaternionをrpyに変換
	tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	double r,p,w;
	tf2::Matrix3x3(quat).getRPY(r,p,w);

	//Twistに格納
	geometry_msgs::Twist twist;
	twist.linear.x = pose.pose.position.x;
	twist.linear.y = pose.pose.position.y;
	twist.linear.z = pose.pose.position.z;
	twist.angular.x = r;
	twist.angular.y = p;
	twist.angular.z = w;
	return twist;
}

int main(int argc, char* argv[])
{
	//ROS初期化
	ros::init(argc, argv, "predict_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate timer(100);

	//リスナー宣言
	PoseListener dirt_pose_listener;
	PoseListener broom_pose_listener;
	PoseListener goal_pose_listener;

	//サブスクライバー宣言
	ros::Subscriber dirt_pose_subscriber = node.subscribe("/ar_dirt_pose", 1, &PoseListener::call_back, &dirt_pose_listener);
	ros::Subscriber broom_pose_subscriber = node.subscribe("/ar_broom_pose", 1, &PoseListener::call_back, &broom_pose_listener);
	ros::Subscriber goal_pose_subscriber = node.subscribe("/ar_goal_pose", 1, &PoseListener::call_back, &goal_pose_listener);

	//moveit初期化
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");

	//全メッセージが到着するまで待つ
	ROS_INFO_STREAM("Waiting for dirt, goal, broom pose, and hand position.");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_dirt_pose");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_broom_pose");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_goal_pose");
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

	//パブリッシャー宣言
	auto target_velocity_publisher = node.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);

	//dynet初期化
	dynet::DynetParams params;
	params.weight_decay = 0.00001;
	dynet::initialize(params);

	//定数設定
	const unsigned int INPUT_DIM = 24;		//入力データの次元
	const unsigned int OUTPUT_DIM = 6;		//出力データの次元
	const unsigned int L1_DIM = 20;
	const unsigned int L2_DIM = 20;
	const unsigned int L3_DIM = 20;
	
	//パラメータ作成（入力以外）
	dynet::ComputationGraph cg;
	dynet::ParameterCollection model;
	dynet::SimpleSGDTrainer trainer(model);
	////L1
	dynet::Parameter p_W1 = model.add_parameters({L1_DIM, INPUT_DIM});
	dynet::Parameter p_b1 = model.add_parameters({L1_DIM});
	////L2
	dynet::Parameter p_W2 = model.add_parameters({L2_DIM, L1_DIM});
	dynet::Parameter p_b2 = model.add_parameters({L2_DIM});
	/////L3
	dynet::Parameter p_W3 = model.add_parameters({L3_DIM, L2_DIM});
	dynet::Parameter p_b3 = model.add_parameters({L3_DIM});
	/////L4
	dynet::Parameter p_W4 = model.add_parameters({OUTPUT_DIM, L3_DIM});
	dynet::Parameter p_b4 = model.add_parameters({OUTPUT_DIM});

	//ノード作成
	dynet::Expression W1 = dynet::parameter(cg, p_W1);
	dynet::Expression W2 = dynet::parameter(cg, p_W2);
	dynet::Expression W3 = dynet::parameter(cg, p_W3);
	dynet::Expression W4 = dynet::parameter(cg, p_W4);
	dynet::Expression b1 = dynet::parameter(cg, p_b1);
	dynet::Expression b2 = dynet::parameter(cg, p_b2);
	dynet::Expression b3 = dynet::parameter(cg, p_b3);
	dynet::Expression b4 = dynet::parameter(cg, p_b4);

	//入出力設定
	auto x_value_ptr = std::make_shared<std::vector<dynet::real>>();	//センサデータののポインタ（入力）
	////ノード作成
	dynet::Expression x = dynet::input(cg, {INPUT_DIM}, x_value_ptr.get());

	//computation graphの構築
	dynet::Expression z1 = dynet::rectify(W1*x  + b1);
	dynet::Expression z2 = dynet::rectify(W2*z1 + b2);
	dynet::Expression z3 = dynet::rectify(W3*z2 + b3);
	dynet::Expression y_pred = W4*z3 + b4;

	//パラメータを読み出し
	dynet::TextFileLoader loader("/home/robot/program/cpp/sia20_learning/build/tmp.model");
	loader.populate(model);
	
	//computation graphを描画
	//std::cout << "=============================================" << std::endl;
	//cg.print_graphviz();
	//std::cout << "=============================================" << std::endl;

	//motomanのエラー回避のためここで速度０のコマンドを数回送る
	geometry_msgs::Twist initial_cmd_vel;
	initial_cmd_vel.linear.x = 0.0;
	initial_cmd_vel.linear.y = 0.0;
	initial_cmd_vel.linear.z = 0.0;
	initial_cmd_vel.angular.x = 0.0;
	initial_cmd_vel.angular.y = 0.0;
	initial_cmd_vel.angular.z = 0.0;
	for (int i = 0; i < 10; i++) {
		target_velocity_publisher.publish(initial_cmd_vel);
		ros::Duration(0.01).sleep();
		ROS_INFO_STREAM("publish initial cmd_vel");
	}

	while (ros::ok()) {
		//センサデータ更新
		ros::spinOnce();
		timer.sleep();

		//センサデータ受け取り(dirt, goal, broomの順に結合)
		std::vector<double> dirt_pose_vector = twist_to_vector(dirt_pose_listener.data);
		std::vector<double> goal_pose_vector = twist_to_vector(goal_pose_listener.data);
		std::vector<double> broom_pose_vector = twist_to_vector(broom_pose_listener.data);
		std::vector<double> hand_pose_vector = twist_to_vector(pose_to_twist(move_group.getCurrentPose()));
		x_value_ptr->clear();

		//受け取ったデータをベクトルx_value_ptrに格納
		x_value_ptr->insert(x_value_ptr->end(), dirt_pose_vector.begin(), dirt_pose_vector.end());
		x_value_ptr->insert(x_value_ptr->end(), goal_pose_vector.begin(), goal_pose_vector.end());
		x_value_ptr->insert(x_value_ptr->end(), broom_pose_vector.begin(), broom_pose_vector.end());

		//推測を実行し，cmd_velを得る
		cg.forward(y_pred);
		std::vector<dynet::real> cmd_vel = dynet::as_vector(y_pred.value());

		//推測したcmd_velを送る
		geometry_msgs::Twist cmd_vel_msgs;
		cmd_vel_msgs.linear.x = (double)cmd_vel.at(0);
		cmd_vel_msgs.linear.y = (double)cmd_vel.at(1);
		cmd_vel_msgs.linear.z = (double)cmd_vel.at(2);
		cmd_vel_msgs.angular.x = (double)cmd_vel.at(3);
		cmd_vel_msgs.angular.y = (double)cmd_vel.at(4);
		cmd_vel_msgs.angular.z = (double)cmd_vel.at(5);
		target_velocity_publisher.publish(cmd_vel_msgs);
		ROS_INFO_STREAM(cmd_vel_msgs);

		ROS_INFO_STREAM("Publish Once");
	}


	return 0;
}
