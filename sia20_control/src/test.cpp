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
	ros::init(argc, argv, "test_node");
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

	//dynet初期化
	dynet::initialize(argc, argv);

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
	auto x_value_ptr = std::make_shared<std::vector<dynet::real>>();	//センサデータのミニバッチのポインタ（入力）
	//auto y_value_ptr = std::make_shared<std::vector<dynet::real>>();	//教師データのミニバッチのポインタ（出力）
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
	std::cout << "=============================================" << std::endl;
	cg.print_graphviz();
	std::cout << "=============================================" << std::endl;

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

	}

	//推測を実行
	////入出力を作成
	auto one_supervised_csv = supervisor_data_csv.get_random_sampling(1);
	for (int j = 0; j < one_supervised_csv.raw_size(); j++) {
		if (j < 24) {	//入力データ作成
			x_value_ptr->push_back(one_supervised_csv(0, j).get_as_double());
		}
		else {	//出力データ作成
			y_value_ptr->push_back(one_supervised_csv(0, j).get_as_double());
		}
	}
	////入出力を表示
	print_vector(*x_value_ptr, "x_value");
	print_vector(*y_value_ptr, "y_value");
	////推測を実行
	cg.forward(y_pred);
	auto y_pred_vec = dynet::as_vector(y_pred.value());
	print_vector(y_pred_vec, "y_pred");

	return 0;
}
