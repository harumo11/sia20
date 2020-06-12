#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <string>
#include <fanda/Csv.hpp>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "repeat_node");
	ros::NodeHandle node_handler;

	CSV::CsvFile csv("/home/robot/catkin_ws/src/sia20/sia20_control/log/test/log_6.csv");
	if (!csv.is_open()) {
		std::cout << "||| can not open csv file" << std::endl;
		std::exit(-1);
	}

	// make publisher
	ros::Publisher publisher = node_handler.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);
	ros::Rate timer(40);
	geometry_msgs::Twist twist_buffer;
	twist_buffer.linear.x = 0.0;
	twist_buffer.linear.y = 0.0;
	twist_buffer.linear.z = 0.0;
	twist_buffer.angular.x = 0.0;
	twist_buffer.angular.y = 0.0;
	twist_buffer.angular.z = 0.0;
	for (int i = 0; i < 40; i++) {
		publisher.publish(twist_buffer);
		timer.sleep();
		ROS_INFO_STREAM("initial publish");
	}
	
	for (int i = 0; i < csv.collumn_size(); i++) {
		twist_buffer.linear.x=csv(i,0).get_as_double();
		twist_buffer.linear.y=csv(i,1).get_as_double();
		twist_buffer.linear.z=csv(i,2).get_as_double();
		twist_buffer.angular.x=csv(i,3).get_as_double();
		twist_buffer.angular.y=csv(i,4).get_as_double();
		twist_buffer.angular.z=csv(i,5).get_as_double();
		ROS_INFO_STREAM(twist_buffer);

		publisher.publish(twist_buffer);
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
