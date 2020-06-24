// このプログラムはOpenCV arucoのARマーカを読み取り
// 最終的にtarget,broomそしてgoalの座標をrosに流すものです．
//
// dirt id : 0
// goal id : 1
// broom id : 2
//
// さらに，砂の中心を計算します．


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char* argv[])
{
	// cv_bridgeと併用するとファイルが読み込めないことがわかった
	// カメラのキャリブレーションデータを読み込む
	// intrinsic matrixを読み込み(3,3)
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 5.7676473334939226e+02, 0.0, 3.2108661199975330e+02, 0.0, 5.7659577304127572e+02, 2.3379326055411508e+02, 0.0, 0.0, 1.0);
	// distortion matrixを読み込み(1,5)
	cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << -1.2974438965267243e-01, 5.8147428099409670e-01,-6.9962431415323987e-03, 3.6420062709177823e-03, -1.1958039271165244e+00) ;
	// ARマーカの辞書を設定
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	// カメラを起動
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		ROS_ERROR_STREAM("Can't open the camera. exit");
		std::exit(-1);
	}
	cv::Mat frame;
	cv::Mat src;

	// ros初期化
	ros::init(argc, argv, "ar_pose_estimation_node");
	ros::NodeHandle node;
	ros::Publisher broom_pose_publisher = node.advertise<geometry_msgs::Twist>("ar_broom_pose", 1);	//箒のarマーカの位置を流すパブリッシャ
	ros::Publisher dirt_pose_publisher = node.advertise<geometry_msgs::Twist>("ar_dirt_pose", 1);	//ゴミのarマーカの位置を流すパブリッシャ
	ros::Publisher goal_pose_publisher = node.advertise<geometry_msgs::Twist>("ar_goal_pose", 1);	//ゴールのarマーカの位置を流すパブリッシャ
	geometry_msgs::Twist dirt_pose, broom_pose, goal_pose;
	image_transport::ImageTransport it(node);
	image_transport::Publisher image_pub = it.advertise("ar_image", 10);
	cv_bridge::CvImage ros_image;
	ros_image.header.stamp = ros::Time::now();
	ros_image.header.frame_id = "ar_image";
	ros_image.encoding = "bgr8";

	while (ros::ok()) {
		// カメラから画像を取得
		cap >> src;
		frame = src;

		// ARマーカをカメラ画像から取得
		std::vector<int> ids;	// 発見したARマーカのIDが入る
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(frame, dictionary, corners, ids);
		
		if (ids.size() > 0) {
			// 検出したマーカの位置と姿勢を推定
			const double marker_side_length = 0.05;	// 印刷したARマーカの一辺の長さ. 8cm
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, marker_side_length, camera_matrix, dist_coeffs, rvecs, tvecs);
			cv::aruco::drawDetectedMarkers(src, corners, ids);
			const double axis_length = 0.1;	// 描画する軸の長さ 10cm
			for (int i = 0; i < (int)ids.size(); i++) {
				cv::aruco::drawAxis(src, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], axis_length);
			}

			// idsの中から0(dirt)を見つける
			auto find_dirt_itr = std::find(ids.begin(), ids.end(), 0);
			if (find_dirt_itr == ids.end()) {
				ROS_WARN_STREAM("Couldn't find dirt marker");
			}
			else {
				int dirt_order_in_ids = std::distance(ids.begin(), find_dirt_itr);
				dirt_pose.linear.x = tvecs.at(dirt_order_in_ids)[0];
				dirt_pose.linear.y = tvecs.at(dirt_order_in_ids)[1];
				dirt_pose.linear.z = tvecs.at(dirt_order_in_ids)[2];
				dirt_pose.angular.x = rvecs.at(dirt_order_in_ids)[0];
				dirt_pose.angular.y = rvecs.at(dirt_order_in_ids)[1];
				dirt_pose.angular.z = rvecs.at(dirt_order_in_ids)[2];
			}
			// idsの中から1(broom)を見つける
			auto find_broom_itr = std::find(ids.begin(), ids.end(), 1);
			if (find_broom_itr == ids.end()) {
				ROS_WARN_STREAM("Couldn't find broom marker");
			}
			else {
				int broom_order_in_ids = std::distance(ids.begin(), find_broom_itr);
				broom_pose.linear.x = tvecs.at(broom_order_in_ids)[0];
				broom_pose.linear.y = tvecs.at(broom_order_in_ids)[1];
				broom_pose.linear.z = tvecs.at(broom_order_in_ids)[2];
				broom_pose.angular.x = rvecs.at(broom_order_in_ids)[0];
				broom_pose.angular.y = rvecs.at(broom_order_in_ids)[1];
				broom_pose.angular.z = rvecs.at(broom_order_in_ids)[2];
			}
			// idsの中から2(goal)を見つける
			auto find_goal_itr = std::find(ids.begin(), ids.end(), 2);
			if (find_goal_itr == ids.end()) {
				ROS_WARN_STREAM("Couldn't find goal marker");
			}
			else {
				int goal_order_in_ids = std::distance(ids.begin(), find_goal_itr);
				goal_pose.linear.x = tvecs.at(goal_order_in_ids)[0];
				goal_pose.linear.y = tvecs.at(goal_order_in_ids)[1];
				goal_pose.linear.z = tvecs.at(goal_order_in_ids)[2];
				goal_pose.angular.x = rvecs.at(goal_order_in_ids)[0];
				goal_pose.angular.y = rvecs.at(goal_order_in_ids)[1];
				goal_pose.angular.z = rvecs.at(goal_order_in_ids)[2];
			}

			// 最初の１つだけマーカの座標を表示
			std::cout << "(x, y, z) : " << tvecs.front()[0] << "\t,\t" << tvecs.front()[1] << "\t,\t" << tvecs.front()[2] << std::endl;

			// publish
			dirt_pose_publisher.publish(dirt_pose);
			goal_pose_publisher.publish(goal_pose);
			broom_pose_publisher.publish(broom_pose);
			ROS_INFO_STREAM("Publish once");
		}
		else {
			std::cout << "NO AR marker is detected" << std::endl;
		}

		ros_image.image = src;
		image_pub.publish(ros_image.toImageMsg());
	}

	return 0;
}
