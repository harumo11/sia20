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
	cv::namedWindow("trackbar");
	cv::Mat gray_frame, hsv_frame, opening_frame, closing_frame, contour_frame;
	cv::Mat labeling_frame, labels, stats, centoids;

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
			const double marker_side_length = 0.06;	// 印刷したARマーカの一辺の長さ. 8cm
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, marker_side_length, camera_matrix, dist_coeffs, rvecs, tvecs);
			cv::aruco::drawDetectedMarkers(frame, corners, ids);
			const double axis_length = 0.1;	// 描画する軸の長さ 10cm
			for (int i = 0; i < (int)ids.size(); i++) {
				cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], axis_length);
			}

			// idsの中から1(broom)を見つける
			auto find_broom_itr = std::find(ids.begin(), ids.end(), 2);
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
			auto find_goal_itr = std::find(ids.begin(), ids.end(), 1);
			if (find_goal_itr == ids.end()) {
				ROS_WARN_STREAM("Couldn't find goal marker");
			}
			else {
				// goalの座標を埋める
				int goal_order_in_ids = std::distance(ids.begin(), find_goal_itr);
				goal_pose.linear.x = tvecs.at(goal_order_in_ids)[0];
				goal_pose.linear.y = tvecs.at(goal_order_in_ids)[1];
				goal_pose.linear.z = tvecs.at(goal_order_in_ids)[2];
				goal_pose.angular.x = rvecs.at(goal_order_in_ids)[0];
				goal_pose.angular.y = rvecs.at(goal_order_in_ids)[1];
				goal_pose.angular.z = rvecs.at(goal_order_in_ids)[2];
				
				// dirtの座標を埋める．goalと同じ平面上にあるので，とりあえずgoalの座標で埋める．
				dirt_pose = goal_pose;
			}

			//　飴を見つけるための処理
			static int hsv_h_min = 29;
			static int hsv_h_max = 70;
			static int hsv_s_min = 37; 
			static int hsv_s_max = 255;
			static int hsv_v_min = 0;
			static int hsv_v_max = 255;
			cv::createTrackbar("hsv_h_min", "trackbar", &hsv_h_min, 255);
			cv::createTrackbar("hsv_h_max", "trackbar", &hsv_h_max, 255);
			cv::createTrackbar("hsv_s_min", "trackbar", &hsv_s_min, 255);
			cv::createTrackbar("hsv_s_max", "trackbar", &hsv_s_max, 255);
			cv::createTrackbar("hsv_v_min", "trackbar", &hsv_v_min, 255);
			cv::createTrackbar("hsv_v_max", "trackbar", &hsv_v_max, 255);
			static int element_close_kernel_size = 10;
			static int element_open_kernel_size = 2;
			cv::createTrackbar("closing_kernel_size", "trackbar", &element_close_kernel_size, 10);
			cv::createTrackbar("opening_kernel_size", "trackbar", &element_open_kernel_size, 10);
			cv::cvtColor(src, hsv_frame, cv::COLOR_BGR2HSV);
			cv::inRange(hsv_frame, cv::Scalar(hsv_h_min, hsv_s_min, hsv_v_min), cv::Scalar(hsv_h_max, hsv_s_max, hsv_v_max), hsv_frame);
			cv::Mat element_close(element_close_kernel_size, element_close_kernel_size, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
			cv::Mat element_open(element_open_kernel_size, element_open_kernel_size, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
			cv::morphologyEx(hsv_frame, opening_frame, cv::MORPH_OPEN, element_open, cv::Point(1, 1), 3);
			cv::morphologyEx(opening_frame, closing_frame, cv::MORPH_CLOSE, element_close, cv::Point(-1, -1), 3);

			// ラベリング
			cv::connectedComponentsWithStats(closing_frame, labels, stats, centoids);
			labeling_frame = closing_frame;
			cv::cvtColor(labeling_frame, labeling_frame, cv::COLOR_GRAY2BGR);
			int max_area_index = 0;
			int max_area = 0;
			for (int i = 0; i < stats.rows; i++) {
				int x = stats.at<int>(cv::Point(0,i));
				int y = stats.at<int>(cv::Point(1,i));
				int w = stats.at<int>(cv::Point(2,i));
				int h = stats.at<int>(cv::Point(3,i));

				int area = w * h;
				std::cout << "area : " << area << std::endl;
				if ((area < 300000) && (area > max_area)) {
					max_area_index = i;
					max_area = area;
				}

				//cv::Scalar color(255, 0, 0);
				//cv::Rect rect(stats.at<int>(cv::Point(0, i)),
				//			  stats.at<int>(cv::Point(1, i)), 
				//			  stats.at<int>(cv::Point(2, i)),
				//			  stats.at<int>(cv::Point(3, i)));
				//cv::rectangle(labeling_frame, rect, color);
			}
			//　最大面積のラベルのバウンディングボックスのみ表示
			std::cout << "Max area index : " << max_area_index << std::endl;
			cv::Scalar color(255, 0, 0);
			int x = stats.at<int>(cv::Point(0, max_area_index));
			int y = stats.at<int>(cv::Point(1, max_area_index));
			int w = stats.at<int>(cv::Point(2, max_area_index));
			int h = stats.at<int>(cv::Point(3, max_area_index));
			cv::Rect rect(x,y,w,h);
			std::cout << "rect area : " << rect.area() << " x : " << x << " y : " << y << std::endl;
			cv::rectangle(labeling_frame, rect, color);
			cv::Vec2d dirt_center = centoids.at<double>(max_area_index);
			// 飴の重心を計算
			dirt_pose.linear.x = (rect.tl().x + rect.br().x)/2.0;
			dirt_pose.linear.y = (rect.tl().y + rect.br().y)/2.0;
			cv::circle(labeling_frame, {(int)dirt_pose.linear.x, (int)dirt_pose.linear.y}, 5, {255, 0, 0}, -1);
			// 深層学習のために正規化する．
			// 205 pixel ==> 0.205 のように1*10^-2する
			dirt_pose.linear.x *= 0.001;
			dirt_pose.linear.y *= 0.001;

			for (int i = 0; i < stats.rows; i++) {
				double *param = centoids.ptr<double>(i);
				int x = static_cast<int>(param[0]);
				int y = static_cast<int>(param[1]);
				cv::circle(labeling_frame, cv::Point(x,y), 3, cv::Scalar(0,0,255), -1);
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

		ros_image.image = frame;
		image_pub.publish(ros_image.toImageMsg());

		// 表示
		cv::imshow("hsv", hsv_frame);
		cv::imshow("closing", closing_frame);
		cv::imshow("opening", opening_frame);
		cv::imshow("result", src);
		cv::imshow("labeld", labeling_frame);
		cv::waitKey(1);
	}

	return 0;
}
