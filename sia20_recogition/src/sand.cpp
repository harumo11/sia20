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
		cap >> src;
		cv::Mat gray_frame, hsv_frame, opening_frame, closing_frame, contour_frame;
		cv::cvtColor(src, hsv_frame, cv::COLOR_BGR2HSV);
		cv::inRange(hsv_frame, cv::Scalar(27, 45, 0), cv::Scalar(62, 255, 255), hsv_frame);
		cv::Mat element_close(2, 2, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
		cv::morphologyEx(hsv_frame, closing_frame, cv::MORPH_OPEN, element_close, cv::Point(-1, -1), 3);
		cv::Mat element_open(3, 3, CV_8U, cv::Scalar::all(255));	//morphingのための構造体
		cv::morphologyEx(closing_frame, opening_frame, cv::MORPH_CLOSE, element_open, cv::Point(-1, -1), 3);

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
		cv::bitwise_not(blob_frame, blob_frame);
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
		//cv::imshow("contours", contour_frame);
		cv::waitKey(27);
	}
	return 0;
}
