# SIA20 project

A goal of sia20 package is to provide functions in order to control sia20(d/f).

## packages

- sia20_description
	This sia20_description subpackage includes foundation of sia20 like 
	```xacro and urdf```, ```robot cad file(stl)```, and so on.
	If you are not familier with ROS system or this package, please start from this subpackage.

- sia20_control
	This sia20_control subpackage includes the configurations of actuator of sia20.
	If you would like to chagne the configuration, like ```pid gain```, please visit this subpackage.

## Tutorial

### 1. Rviz (Getting started in 30 sec)
First of all. take a look sia20 in rviz.

```
roslaunch sia20_description display.launch
```

### 2. Gazebo (Simulator)
As a next step, launch a simulation and move sia20.

```
roslaunch sia20_description gazebo.launch
```

Pleaes wait until gazebo launch.

Let's look the list of rostopic
```
rostopic list
```

You should look as below.
```
harumo@harumo-thinkpad-x1:~$ rostopic list
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/rosout
/rosout_agg
/sia20/joint_b_position_controller/command
/sia20/joint_b_position_controller/pid/parameter_descriptions
/sia20/joint_b_position_controller/pid/parameter_updates
/sia20/joint_b_position_controller/state
/sia20/joint_e_position_controller/command
/sia20/joint_e_position_controller/pid/parameter_descriptions
/sia20/joint_e_position_controller/pid/parameter_updates
/sia20/joint_e_position_controller/state
/sia20/joint_l_position_controller/command
/sia20/joint_l_position_controller/pid/parameter_descriptions
/sia20/joint_l_position_controller/pid/parameter_updates
/sia20/joint_l_position_controller/state
/sia20/joint_r_position_controller/command
/sia20/joint_r_position_controller/pid/parameter_descriptions
/sia20/joint_r_position_controller/pid/parameter_updates
/sia20/joint_r_position_controller/state
/sia20/joint_s_position_controller/command
/sia20/joint_s_position_controller/pid/parameter_descriptions
/sia20/joint_s_position_controller/pid/parameter_updates
/sia20/joint_s_position_controller/state
/sia20/joint_states
/sia20/joint_t_position_controller/command
/sia20/joint_t_position_controller/pid/parameter_descriptions
/sia20/joint_t_position_controller/pid/parameter_updates
/sia20/joint_t_position_controller/state
/sia20/joint_u_position_controller/command
/sia20/joint_u_position_controller/pid/parameter_descriptions
/sia20/joint_u_position_controller/pid/parameter_updates
/sia20/joint_u_position_controller/state
/tf
/tf_static
```

Let's look each joints state such as angle, velocity, effort(stands for moment[N/m]).

```
rostopic echo /sia20/joint_states
```

Let's move joint using following command.
```
rostopic pub -1 /sia20/joint_s_position_controller/command std_msgs/Float64 "data 1.5"
```
You can see that sia20 moves in gazebo.
