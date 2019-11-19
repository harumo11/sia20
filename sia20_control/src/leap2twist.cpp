#include <iostream>
#include <thread>
#include <iomanip>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include </home/harumo/catkin_ws/devel/include/leap_motion/leapros.h>

class Leap2Twist {
	public:
		void call_back(const leap_motion::leapros data);
		geometry_msgs::Twist get();
		geometry_msgs::Twist twist_data_t;
		geometry_msgs::Twist twist_data_t_1;
		const double scale_factor = 1.0;
		const double angle_bias_x = 0;
		const double angle_bias_y = 0;
		const double angle_bias_z = 0;
};

void Leap2Twist::call_back(const leap_motion::leapros data){
	//transition
	this->twist_data_t_1 = this->twist_data_t;

	//position
	this->twist_data_t.linear.x = this->scale_factor * data.palmpos.x;
	this->twist_data_t.linear.y = this->scale_factor * data.palmpos.y;
	this->twist_data_t.linear.z = this->scale_factor * data.palmpos.z;

	//angle
	this->twist_data_t.angular.x = data.ypr.x + this->angle_bias_x;
	this->twist_data_t.angular.y = data.ypr.y + this->angle_bias_y;
	this->twist_data_t.angular.z = data.ypr.z + this->angle_bias_z;
}

geometry_msgs::Twist Leap2Twist::get(){
	geometry_msgs::Twist diff;
	diff.linear.x = this->twist_data_t.linear.x - this->twist_data_t_1.linear.x;
	diff.linear.y = this->twist_data_t.linear.y - this->twist_data_t_1.linear.y;
	diff.linear.z = this->twist_data_t.linear.z - this->twist_data_t_1.linear.z;
	diff.angular.x = this->twist_data_t.angular.x - this->twist_data_t_1.angular.x;
	diff.angular.y = this->twist_data_t.angular.y - this->twist_data_t_1.angular.y;
	diff.angular.z = this->twist_data_t.angular.z - this->twist_data_t_1.angular.z;

	return diff;
}

void moniter_enter(std::shared_ptr<bool> pushed_enter){

	while (ros::ok()) {
		if (std::cin.get()) {
			*pushed_enter = !(*pushed_enter);
			std::cout << "enterd : " << std::boolalpha << *pushed_enter << std::endl;
		}
	}
}

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "leap2twist_node");

	//make node_handler
	ros::NodeHandle nodehander;
	ros::Rate timer(100);

	//make subscriber
	Leap2Twist leap_to_twist;
	ros::Subscriber subscriber = nodehander.subscribe("/leapmotion/data", 1, &Leap2Twist::call_back, &leap_to_twist);

	//make publisher
	ros::Publisher publisher_enter = nodehander.advertise<std_msgs::Bool>("/leap2twist/enter", 1);
	ros::Publisher publisher_leap  = nodehander.advertise<geometry_msgs::Twist>("/leap2twist/leap", 1);

	//make thread
	auto pushed_enter_prt = std::make_shared<bool>(false);
	std::thread enter_thread(moniter_enter, pushed_enter_prt);

	while (ros::ok()) {
		ROS_INFO_STREAM(leap_to_twist.get());
		
		//publish leap
		publisher_leap.publish(leap_to_twist.get());
		//publish bool
		std_msgs::Bool is_pushed_msgs;
		is_pushed_msgs.data = *pushed_enter_prt;;
		publisher_enter.publish(is_pushed_msgs);

		ros::spinOnce();
		timer.sleep();
	}

	enter_thread.join();
	
	return 0;
}
