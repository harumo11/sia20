#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char* argv[])
{
	
	//ros init
	ros::init(argc, argv, "test_get_state");

	//node handle
	ros::NodeHandle node_handle;

	//client
	ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	//request
	gazebo_msgs::GetModelState srv;
	srv.request.model_name = "hausing";

	if (client.call(srv)) {
		ROS_INFO_STREAM(srv.response.pose);
	}
	else {
		std::cout << "Failed to call" << std::endl;
	}

	return 0;
}
