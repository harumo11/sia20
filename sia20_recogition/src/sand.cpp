#include <iostream>
#include <opencv2/opencv.hpp>




int main(int argc, char const* argv[])
{
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cout << "|||Error. can't open camera" << std::endl;
	}

	cv::namedWindow("trackbar");
	cv::Mat src;
	std::cout << "OpenCV version : " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << std::endl;

	while (true) {
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
		cap >> src;
		cv::Mat gray_frame, hsv_frame, opening_frame, closing_frame, contour_frame;
		cv::cvtColor(src, hsv_frame, cv::COLOR_BGR2HSV);
		cv::inRange(hsv_frame, cv::Scalar(hsv_h_min, hsv_s_min, hsv_v_min), cv::Scalar(hsv_h_max, hsv_s_max, hsv_v_max), hsv_frame);
		cv::Mat element_close(element_close_kernel_size, element_close_kernel_size, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
		cv::Mat element_open(element_open_kernel_size, element_open_kernel_size, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
		cv::morphologyEx(hsv_frame, opening_frame, cv::MORPH_OPEN, element_open, cv::Point(1, 1), 3);
		cv::morphologyEx(opening_frame, closing_frame, cv::MORPH_CLOSE, element_close, cv::Point(-1, -1), 3);

		// ラベリング
		cv::Mat labeling_frame, labels, stats, centoids;
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
		cv::circle(labeling_frame, (rect.tl() + rect.br())/2.0, 5, {255, 0, 0}, -1);

		for (int i = 0; i < stats.rows; i++) {
			double *param = centoids.ptr<double>(i);
			int x = static_cast<int>(param[0]);
			int y = static_cast<int>(param[1]);
			cv::circle(labeling_frame, cv::Point(x,y), 3, cv::Scalar(0,0,255), -1);
		}


		static int min_thres = 0;
		static int max_thres = 300;
		static int min_area = 1;
		cv::createTrackbar("min_thres", "trackbar", &min_thres, 100);
		cv::createTrackbar("max_thres", "trackbar", &max_thres, 300);
		cv::createTrackbar("min_area", "trackbar", &min_area, 500);
		std::cout << min_thres << std::endl;
		cv::SimpleBlobDetector::Params blob_detector_params;
		blob_detector_params.minThreshold = min_thres;
		blob_detector_params.maxThreshold = max_thres;
		blob_detector_params.minArea = min_area;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blob_detector_params);
		std::vector<cv::KeyPoint> key_vector;
		detector->detect(opening_frame, key_vector);
		cv::Mat blob_frame = opening_frame;
		//cv::bitwise_not(blob_frame, blob_frame);
		cv::drawKeypoints(blob_frame, key_vector, blob_frame, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);



		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(opening_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
		std::vector<std::vector<cv::Point>> hull(contours.size());
		for (size_t i = 0; i < contours.size(); i++){
			cv::convexHull(contours[i], hull[i]);
		}

		for (size_t i = 0; i < contours.size(); i++){
			cv::Scalar color = {255, 255, 255};
			cv::drawContours(src, contours, (int)i, color);
			//cv::drawContours(src, hull, (int)i, color);
		}

		cv::imshow("hsv", hsv_frame);
		cv::imshow("closing", closing_frame);
		cv::imshow("opening", opening_frame);
		cv::imshow("result", src);
		cv::imshow("blob", blob_frame);
		cv::imshow("labeled", labeling_frame);
		//cv::imshow("contours", contour_frame);
		cv::waitKey(27);
	}
	return 0;
}
