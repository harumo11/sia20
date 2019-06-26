cmake_minimum_required(VERSION 2.8.3)

project(sia20_control)
set(CMAKE_EXPORT_COMPILE_COMMAND ON)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
add_executable(sequential_ik_solver src/sequential_ik_solver.cpp)
add_executable(ds4totwist src/ds4totwist.cpp)
add_executable(ik_test src/ik_test.cpp)

target_link_libraries(joint_trajectory_publisher ${catkin_LIBRARIES})
target_link_libraries(sequential_ik_solver ${catkin_LIBRARIES})
target_link_libraries(ds4totwist ${catkin_LIBRARIES})
target_link_libraries(ik_test ${catkin_LIBRARIES})
