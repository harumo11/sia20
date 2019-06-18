#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class Joy2Twist {
	
	public:
		void call_back(const sensor_msgs::Joy joy_msgs);
		geometry_msgs::Twist get();
		geometry_msgs::Twist twist_msgs;
		double scale_factor_linear = 0.05;
		double scale_factor_angular = 0.05;
};

void Joy2Twist::call_back(const sensor_msgs::Joy joy_msgs){
	// initialize and clear
	this->twist_msgs = geometry_msgs::Twist();

	// for representing z 
	bool is_button_pushed = false;
	for (auto&& button : joy_msgs.buttons){
		if (button > 0) {
			is_button_pushed = true;
		}
	}

	if (is_button_pushed) {
		// analog stic left (x axis) + any button -> z
		this->twist_msgs.linear.z = scale_factor_linear * joy_msgs.axes[1];
	}
	else {
		// analog stic left (x axis) -> x
		this->twist_msgs.linear.x = scale_factor_linear * joy_msgs.axes[1];
		// analog stic left (y axis) -> y
		this->twist_msgs.linear.y = scale_factor_linear * joy_msgs.axes[0];
	}

	// roll
	this->twist_msgs.angular.x = scale_factor_angular * joy_msgs.axes[3];
	// pitch
	this->twist_msgs.angular.y = scale_factor_angular * joy_msgs.axes[4];
	// yaw
	this->twist_msgs.angular.z = scale_factor_angular * ( ( joy_msgs.axes[2] - joy_msgs.axes[5] ) * (-1) / 2);


	ROS_INFO_STREAM("x : " << this->twist_msgs.linear.x << "\t" <<
			        "y : " << this->twist_msgs.linear.y << "\t" << 
					"z : " << this->twist_msgs.linear.z << "\t" << 
					"roll : " << this->twist_msgs.angular.x << "\t" << 
					"pitch : " << this->twist_msgs.angular.y << "\t" << 
					"yaw : " << this->twist_msgs.angular.z << std::endl);

}

geometry_msgs::Twist Joy2Twist::get(){
	return this->twist_msgs;
}

int main(int argc, char* argv[])
{
	// ros initialization
	ros::init(argc, argv, "ds4totwist_node");

	// make node handle
	ros::NodeHandle node_handler;

	// make subscriber
	Joy2Twist joy_to_twist;
	ros::Subscriber subscriber = node_handler.subscribe("/joy", 1, &Joy2Twist::call_back, &joy_to_twist);

	// make publisher
	ros::Publisher publisher = node_handler.advertise<geometry_msgs::Twist>("target_velocity", 1);
	ros::Rate timer(100);

	while (ros::ok()) {
		ROS_INFO_STREAM(joy_to_twist.get());
		geometry_msgs::Twist twist_buffer = joy_to_twist.get();
		publisher.publish(twist_buffer);
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
