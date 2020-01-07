cmake_minimum_required(VERSION 2.8.3)

project(sia20_control)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
add_definitions("-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  gazebo_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
#add_executable(joint_trajectory_publisher_ik src/joint_trajectory_publisher_ik.cpp)
#add_executable(joint_trajectory_publisher_multi src/joint_trajectory_publisher_multi.cpp)
#add_executable(joint_trajectory_publisher_pouring src/joint_trajectory_publisher_pouring.cpp)
add_executable(sequential_ik_solver src/sequential_ik_solver.cpp)
add_executable(ds4totwist src/ds4totwist.cpp)
add_executable(ik_test src/ik_test.cpp)
add_executable(move_init_pose src/move_init_pose.cpp)
#add_executable(leap2twist src/leap2twist.cpp)
add_executable(test_get_coord src/test_get_coord.cpp)
add_executable(test_get_coord_srv src/test_get_coord_srv.cpp)
#add_executable(sim_manager src/sim_manager.cpp)
add_executable(get_joint_angle src/get_joint_angle.cpp)
#add_executable(moveit_get_pose src/moveit_get_pose.cpp)
#add_executable(pouring_trajectory_action_client src/pouring_trajectory_action_client.cpp)
add_executable(pose_follow src/pose_follow.cpp)
add_executable(pose_follow_fast src/pose_follow_fast.cpp)
add_executable(pose_follow_planner src/pose_follow_planner.cpp)
add_executable(pose_follow_executer src/pose_follow_executer.cpp)

target_link_libraries(joint_trajectory_publisher ${catkin_LIBRARIES})
#target_link_libraries(joint_trajectory_publisher_ik ${catkin_LIBRARIES})
#target_link_libraries(joint_trajectory_publisher_multi ${catkin_LIBRARIES})
#target_link_libraries(joint_trajectory_publisher_pouring ${catkin_LIBRARIES})
target_link_libraries(sequential_ik_solver ${catkin_LIBRARIES})
target_link_libraries(ds4totwist ${catkin_LIBRARIES})
target_link_libraries(ik_test ${catkin_LIBRARIES})
target_link_libraries(move_init_pose ${catkin_LIBRARIES})
#target_link_libraries(leap2twist ${catkin_LIBRARIES})
target_link_libraries(test_get_coord ${catkin_LIBRARIES})
target_link_libraries(test_get_coord_srv ${catkin_LIBRARIES})
#target_link_libraries(sim_manager ${catkin_LIBRARIES})
target_link_libraries(get_joint_angle ${catkin_LIBRARIES})
#target_link_libraries(moveit_get_pose ${catkin_LIBRARIES})
#target_link_libraries(pouring_trajectory_action_client ${catkin_LIBRARIES} fanda)
target_link_libraries(pose_follow ${catkin_LIBRARIES})
target_link_libraries(pose_follow_fast ${catkin_LIBRARIES})
target_link_libraries(pose_follow_planner ${catkin_LIBRARIES})
target_link_libraries(pose_follow_executer ${catkin_LIBRARIES})
