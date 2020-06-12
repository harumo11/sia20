#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <fanda/Tcp.hpp>


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sakakibara_node");
    std::string str;
	ros::NodeHandle node_handler;

  

	// make publisher
	ros::Publisher publisher = node_handler.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);
	ros::Rate timer(1000);

	while (ros::ok()) {
		geometry_msgs::Twist twist_buffer;
		twist_buffer.linear.x=0.0;
		twist_buffer.linear.y=0.0;
		twist_buffer.linear.z=std::stod(str);
		twist_buffer.angular.x=0.0;
		twist_buffer.angular.y=0.0;
		twist_buffer.angular.z=0.0;

		ROS_INFO_STREAM(twist_buffer);

		publisher.publish(twist_buffer);
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
