cmake_minimum_required(VERSION 2.8.3)

project(sia20_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
target_link_libraries(joint_trajectory_publisher ${catkin_LIBRARIES})
