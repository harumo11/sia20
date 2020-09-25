//このプログラムはdynetを使用して学習した箒の使用方法を実際に推測し，実行するための
//プログラムです．学習済みモデルを読み込み，ニューラルネットを構築し
//そこにリアルタイムで得たセンサーからの情報を入力として流し込み
//出力として手先の速度を得ます．その後，ROSでその速度をpublishします．
//
//入力&出力は下記のURLを参照とのこと（レプトリノ含む）
//https://harumo11.github.io/sia20/training_data/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <memory>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include <fanda/Csv.hpp>
#include <dynet/io.h>
#include <dynet/training.h>
#include <dynet/expr.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <Gpop/Series.hpp>


void twist_zero_clear(geometry_msgs::Twist& msgs){
	msgs.linear.x = 0;
	msgs.linear.y = 0;
	msgs.linear.z = 0;
	msgs.angular.x = 0;
	msgs.angular.y = 0;
	msgs.angular.z = 0;
}

class SensorListener {
	public:
		std_msgs::Float32MultiArray data;
		void call_back(const std_msgs::Float32MultiArray msgs){ this->data = msgs; }
};

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

void update_hand_pose_arrays(std::array<std::array<double, 6>, 3> hand_pose_arrays, const geometry_msgs::PoseStamped current_pose){
	// 最新の値を使えるように変換
	auto latest_hand_pose_vector = twist_to_vector(pose_to_twist(current_pose));
	// １番めの配列を２番めに移す
	hand_pose_arrays.at(2) = hand_pose_arrays.at(1);
	// ０番目の配列を１番めに移す
	hand_pose_arrays.at(1) = hand_pose_arrays.at(0);
	// 最新の配列を０番目に入れる
	int count = 0;
	for (auto&& pose : hand_pose_arrays.at(0)){
		pose = latest_hand_pose_vector.at(count);
		count++;
	}
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
	SensorListener sensor_listener;

	//サブスクライバー宣言
	ros::Subscriber dirt_pose_subscriber = node.subscribe("/ar_dirt_pose", 1, &PoseListener::call_back, &dirt_pose_listener);
	ros::Subscriber broom_pose_subscriber = node.subscribe("/ar_broom_pose", 1, &PoseListener::call_back, &broom_pose_listener);
	ros::Subscriber goal_pose_subscriber = node.subscribe("/ar_goal_pose", 1, &PoseListener::call_back, &goal_pose_listener);
	ros::Subscriber leptorino_subscriber = node.subscribe("/sensor_data", 1, &SensorListener::call_back, &sensor_listener);

	//moveit初期化
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");

	//全メッセージが到着するまで待つ
	ROS_INFO_STREAM("Waiting for dirt, goal, broom pose, and hand position.");
	ROS_INFO_STREAM("ar_dirt_pose message is searching");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_dirt_pose");
	ROS_INFO_STREAM("ar_broom_pose message is searching");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_broom_pose");
	ROS_INFO_STREAM("ar_goal_pose message is searching");
	ros::topic::waitForMessage<geometry_msgs::Twist>("ar_goal_pose");
	ROS_INFO_STREAM("joint_states message is searching");
	ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
	ROS_INFO_STREAM("sensor_data message is searching");
	ros::topic::waitForMessage<std_msgs::Float32MultiArray>("sensor_data");
	ROS_INFO_STREAM("all message is found");

	//パブリッシャー宣言
	auto target_velocity_publisher = node.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);

	//dynet初期化
	dynet::DynetParams params;
	params.weight_decay = 0;
	params.cpu_requested = true;
	dynet::initialize(params);

	//定数設定
	const unsigned int INPUT_LAYER_DIMENSION = 37;		//入力データの次元
	const unsigned int HIDDEN_LAYER_DIMENSION = 20;		//中間層の次元
	const unsigned int OUTPUT_LAYER_DIMENSION = 6;		//出力データの次元
	
	//パラメータ作成（入力以外）
	dynet::ComputationGraph cg;
	dynet::ParameterCollection model;
	dynet::SimpleSGDTrainer trainer(model);
	////L1
	dynet::Parameter p_W1 = model.add_parameters({HIDDEN_LAYER_DIMENSION, INPUT_LAYER_DIMENSION});
	dynet::Parameter p_b1 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	////L2
	dynet::Parameter p_W2 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b2 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	/////L3
	dynet::Parameter p_W3 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b3 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	/////L4
	dynet::Parameter p_W4 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b4 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	/////L5
	dynet::Parameter p_W5 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b5 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	////L6
	dynet::Parameter p_W6 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b6 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	////L7
	dynet::Parameter p_W7 = model.add_parameters({HIDDEN_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b7 = model.add_parameters({HIDDEN_LAYER_DIMENSION});
	////L8
	dynet::Parameter p_W8 = model.add_parameters({OUTPUT_LAYER_DIMENSION, HIDDEN_LAYER_DIMENSION});
	dynet::Parameter p_b8 = model.add_parameters({OUTPUT_LAYER_DIMENSION});
	
	//ノード作成
	dynet::Expression W1 = dynet::parameter(cg, p_W1);
	dynet::Expression W2 = dynet::parameter(cg, p_W2);
	dynet::Expression W3 = dynet::parameter(cg, p_W3);
	dynet::Expression W4 = dynet::parameter(cg, p_W4);
	dynet::Expression W5 = dynet::parameter(cg, p_W5);
	dynet::Expression W6 = dynet::parameter(cg, p_W6);
	dynet::Expression W7 = dynet::parameter(cg, p_W7);
	dynet::Expression W8 = dynet::parameter(cg, p_W8);
	dynet::Expression b1 = dynet::parameter(cg, p_b1);
	dynet::Expression b2 = dynet::parameter(cg, p_b2);
	dynet::Expression b3 = dynet::parameter(cg, p_b3);
	dynet::Expression b4 = dynet::parameter(cg, p_b4);
	dynet::Expression b5 = dynet::parameter(cg, p_b5);
	dynet::Expression b6 = dynet::parameter(cg, p_b6);
	dynet::Expression b7 = dynet::parameter(cg, p_b7);
	dynet::Expression b8 = dynet::parameter(cg, p_b8);

	//入出力設定
	auto x_value_ptr = std::make_shared<std::vector<dynet::real>>();	//センサデータののポインタ（入力）
	////ノード作成
	dynet::Expression x = dynet::input(cg, {INPUT_LAYER_DIMENSION}, x_value_ptr.get());

	//computation graphの構築
	dynet::Expression z1 = dynet::rectify(W1*x  + b1);
	dynet::Expression z2 = dynet::rectify(W2*z1 + b2);
	dynet::Expression z3 = dynet::rectify(W3*z2 + b3);
	dynet::Expression z4 = dynet::rectify(W4*z3 + b4);
	dynet::Expression z5 = dynet::rectify(W5*z4 + b5);
	dynet::Expression z6 = dynet::rectify(W6*z5 + b6);
	dynet::Expression z7 = dynet::rectify(W7*z6 + b7);
	dynet::Expression y_pred = W8*z7 + b8;

	//パラメータを読み出し
	//dynet::TextFileLoader loader("/home/robot/catkin_ws/src/sia20/sia20_control/model/train3_minibatch_2.model");
	dynet::TextFileLoader loader("/home/robot/program/cpp/sia20_learning/build/long_normal_broom_weight_decay_0_000001_minibatch_size_2.model");
	loader.populate(model);
	
	//computation graphを描画
	//std::cout << "=============================================" << std::endl;
	//cg.print_graphviz();
	//std::cout << "=============================================" << std::endl;


	int iter = 0;
	// hand_pose_arrays which contains hand_pose (t, t-1, t-2)
	std::array<std::array<double, 6>, 3> hand_pose_arrays;
	// hand_pose_arrays initialize
	auto current_hand_pose = move_group.getCurrentPose();
	for (auto iter = std::rbegin(hand_pose_arrays); iter != std::rend(hand_pose_arrays); iter++){
		auto hand_pose_vector = twist_to_vector(pose_to_twist(move_group.getCurrentPose()));
		int counter = 0;
		for (auto&& e : *iter){
			e = hand_pose_vector.at(counter);
			counter++;
		}
		ros::Duration(0.01).sleep();
	}

	//motomanのエラー回避のためここで速度０のコマンドを数回送る
	geometry_msgs::Twist initial_cmd_vel;
	twist_zero_clear(initial_cmd_vel);
	ROS_WARN_STREAM("starting to publish initial cmd_vel");
	for (int i = 0; i < 100; i++) {
		target_velocity_publisher.publish(initial_cmd_vel);
		ros::Duration(0.01).sleep();
	}
	ROS_WARN_STREAM("finishing to publish initial cmd_vel");

	// 推測データ表示用のプロット
	Gpop::Series lin_x_plot("linear x");
	Gpop::Series lin_y_plot("linear y");
	Gpop::Series lin_z_plot("linear z");

	while (ros::ok()) {
		//センサデータ更新
		ros::spinOnce();

		//センサデータ受け取り(dirt, goal, broomの順に結合)
		std::vector<double> dirt_pose_vector = twist_to_vector(dirt_pose_listener.data);
		std::vector<double> goal_pose_vector = twist_to_vector(goal_pose_listener.data);
		std::vector<double> broom_pose_vector = twist_to_vector(broom_pose_listener.data);
		std::vector<double> hand_pose_vector = twist_to_vector(pose_to_twist(move_group.getCurrentPose()));
		double leptorino_datum = sensor_listener.data.data.at(0);

		// update hand_pose_arrays at here
		update_hand_pose_arrays(hand_pose_arrays, move_group.getCurrentPose());
		// 入力ベクトルを空にする
		x_value_ptr->clear();

		//受け取ったデータをベクトルx_value_ptrに格納
		// end effector position t 
		x_value_ptr->insert(x_value_ptr->end(), hand_pose_arrays.at(0).begin(), hand_pose_arrays.at(0).end());
		// end effector position t-1
		x_value_ptr->insert(x_value_ptr->end(), hand_pose_arrays.at(1).begin(), hand_pose_arrays.at(1).end());
		// end effector position t-2
		x_value_ptr->insert(x_value_ptr->end(), hand_pose_arrays.at(2).begin(), hand_pose_arrays.at(2).end());
		// candy position
		x_value_ptr->insert(x_value_ptr->end(), dirt_pose_vector.begin(), dirt_pose_vector.begin()+6);	
		// broom position
		x_value_ptr->insert(x_value_ptr->end(), broom_pose_vector.begin(), broom_pose_vector.begin()+6);
		// dustpan position
		x_value_ptr->insert(x_value_ptr->end(), goal_pose_vector.begin(), goal_pose_vector.begin()+6);
		// leptorino
		x_value_ptr->push_back(leptorino_datum);
		
		std::cout << "|||x value ptr size : " << x_value_ptr->size() << std::endl;
		if (x_value_ptr->size() != 37) {
			std::cerr << "x_value input has invalid dimension. exit()" << std::endl;
			std::exit(1);
		}
		for (int i = 0; i < x_value_ptr->size(); i++) {
			std::cout << "||| input x vector " << x_value_ptr->at(i) << " , ";
		}
		std::cout << std::endl;

		//推測を実行し，cmd_velを得る
		cg.forward(y_pred);
		std::vector<dynet::real> cmd_vel = dynet::as_vector(y_pred.value());
		std::cout << "||| (x,y,z) = " << cmd_vel.at(0) << " , " << cmd_vel.at(1) << " , " << cmd_vel.at(2) << " (r,p,w) = "  << cmd_vel.at(3) << " , " << cmd_vel.at(4) << " , " << cmd_vel.at(5) << std::endl;

		//推測したcmd_velを送る
		geometry_msgs::Twist cmd_vel_msgs;
		cmd_vel_msgs.linear.x = cmd_vel.at(0);
		cmd_vel_msgs.linear.y = cmd_vel.at(1);
		cmd_vel_msgs.linear.z = cmd_vel.at(2);
		cmd_vel_msgs.angular.x = cmd_vel.at(3);
		cmd_vel_msgs.angular.y = cmd_vel.at(4);
		cmd_vel_msgs.angular.z = cmd_vel.at(5);
		target_velocity_publisher.publish(cmd_vel_msgs);
		ROS_INFO_STREAM(cmd_vel_msgs);
		lin_x_plot.plot(cmd_vel.at(0));
		lin_y_plot.plot(cmd_vel.at(1));
		lin_z_plot.plot(cmd_vel.at(2));
		lin_x_plot.pause();
		lin_y_plot.pause();
		lin_z_plot.pause();

		ROS_INFO_STREAM("Publish Once");

		timer.sleep();
	}


	return 0;
}
