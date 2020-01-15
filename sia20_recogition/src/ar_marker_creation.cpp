#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char const* argv[])
{
	const int created_marker_num = 3;
	auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	for (int i = 0; i < created_marker_num; i++) {
		cv::Mat marker_image;
		cv::aruco::drawMarker(dictionary, i, 200, marker_image);
		cv::waitKey(1);
		cv::imshow("marker " + std::to_string(i), marker_image);
		cv::imwrite("/home/robot/catkin_ws/src/sia20/sia20_recogition/marker/marker" + std::to_string(i) + ".png", marker_image);
	}
	return 0;
}
