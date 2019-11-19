#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>

class GazeboModel {
	public:
		void call_back(const gazebo_msgs::ModelStates& model_states_msgs);
		geometry_msgs::Twist get(std::string model_name);
		gazebo_msgs::ModelStates data;
		int size = 0;
};

void GazeboModel::call_back(const gazebo_msgs::ModelStates& model_states_msgs){
	ROS_INFO_STREAM("call_back");
	ROS_INFO_STREAM(model_states_msgs);
	this->data = model_states_msgs;
	this->size = model_states_msgs.name.size();
	ROS_INFO_STREAM("size : " << this->size);
}

geometry_msgs::Twist GazeboModel::get(std::string model_name){

	ROS_INFO_STREAM("get");
	int index = 0;
	for (int i = 0; i < this->size; i++) {
		if (this->data.name[i] == model_name) {
			index = i;
			ROS_INFO_STREAM("found");
			break;
		}
	}

	ROS_INFO_STREAM(data.twist.at(index));
	return this->data.twist[index];
}

int main(int argc, char* argv[])
{
	
	//ros init
	ros::init(argc, argv, "test_get_coord");

	//node hander
	ros::NodeHandle node_handle;

	//subscribe
	GazeboModel gazebo_listener;
	ros::Subscriber sub = node_handle.subscribe("/gazebo/model_states", 1, &GazeboModel::call_back, &gazebo_listener); 

	//timer
	ros::Rate timer(10);

	while (ros::ok()) {
		ros::spinOnce();
		timer.sleep();
		//gazebo_listener.get("hausing");
	}
	return 0;
}
