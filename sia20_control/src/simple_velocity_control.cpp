#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <fstream>
#include <fanda/Tcp.hpp>

class Float32MultiArrayListener {
	public:
		Float32MultiArrayListener();
		std_msgs::Float32MultiArray data;
		std::ofstream log_file;
		void call_back(std_msgs::Float32MultiArray received_data);
		~Float32MultiArrayListener();
};

Float32MultiArrayListener::Float32MultiArrayListener(){
	this->log_file.open("/home/robot/catkin_ws/src/sia20/sia20_recogition/sensor_data.csv");
	if (!log_file.is_open()) {
		ROS_ERROR_STREAM("can't open log file. exit.");
		std::exit(-1);
	}
}

void Float32MultiArrayListener::call_back(std_msgs::Float32MultiArray received_data){
	for (auto e: received_data.data){
		this->log_file << e << ",";
	}
	this->log_file << std::endl;
	ROS_INFO_STREAM("Wrote");
}

Float32MultiArrayListener::~Float32MultiArrayListener(){
	this->log_file.close();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "velocity_cntl_node");
    std::string str;
	ros::NodeHandle node_handler;

	// make publisher
	ros::Publisher publisher = node_handler.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);
	ros::Rate timer(40);

	// make subscriber
	Float32MultiArrayListener listener;
	ros::Subscriber subscriber = node_handler.subscribe("sensor_data", 1, &Float32MultiArrayListener::call_back, &listener);

	// initialize
	geometry_msgs::Twist twist_buffer;
	twist_buffer.linear.x = 0;
	twist_buffer.linear.y = 0;
	twist_buffer.linear.z = 0;
	twist_buffer.angular.x = 0;
	twist_buffer.angular.y = 0;
	twist_buffer.angular.z = 0;
	for (int i = 0; i < 100; i++) {
		publisher.publish(twist_buffer);
		timer.sleep();
	}

	bool should_write_9999 = false;
	for (int i = 0; i < 20; i++) {
		ROS_WARN_STREAM("Iteration " << i );
		for (int itr = 0; itr < 160; itr++) {
			double t = itr * ((2 * M_PI)/160.0);
			double z = -0.06 * std::sin(t);
			
			geometry_msgs::Twist twist_buffer;
			twist_buffer.linear.x = 0.0;
			twist_buffer.linear.y = 0.0;
			twist_buffer.linear.z = z;
			twist_buffer.angular.x = 0.0;
			twist_buffer.angular.y = 0.0;
			twist_buffer.angular.z = 0.0;

			ROS_INFO_STREAM(twist_buffer);

			publisher.publish(twist_buffer);
			ros::spinOnce();
			timer.sleep();
		}
		listener.log_file << 9999 << "," << 9999 << "," << 9999 << "," << 9999 << "," << 9999 << "," << 9999 << std::endl;
	}
	
	return 0;
}
