# SIA20 project

A goal of sia20 package is to provide functions in order to control sia20(d/f).

![demo1](https://github.com/harumo11/sia20/blob/media/gazebo-moveit-rviz.gif)

![graphviz](https://github.com/harumo11/sia20/blob/media/gazebo-moveit-rviz.png)


## Packages

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
rostopic pub -1 /sia20/joint_s_position_controller/command std_msgs/Float64 "data 1.0"
```
You can see that sia20 moves in gazebo.

## How to add a new gripper

reference 
- [gazebo ros control](http://gazebosim.org/tutorials/?tut=ros_control)
- [urdf turorial](http://wiki.ros.org/urdf/Tutorials)

### 0. Make your package

Make your new package. It may be convenient to make new package as subpackage of sia20.

```sh
sia20
  |--sia20_description
  |--sia20_control
  |--your_new_package!!
```

or, you can add new urdf and stl file like following.

```
sia20
  |-- sia20_description
  			|-- urdf
  			|     |-- sia20.urdf
  			|     |-- **your_gripper.urdf**
  			|
  			|-- mesh
  			      |-- **gripper.STL**
```



In order to make new gripper package, type following command.

```sh
mkdir ~/catkin_ws
cd ~/catkin_ws
catkin_create_pkg your_package controller_manager joint_state_controller robot_state_publisher
cd your_package
mkdir config
mkdir launch
```

### 1. Prepare gripper STL or COLLADA file.

Make your gripper CAD file and save as STL file.

### 2. Make new urdf or xacro

Make new urdf that includes sia20's urdf(```sia20_description/sia20.urdf.xacro```).
And write configuration regarding to your gripper.

If you use Solidworks, you can make urdf file using sw_urdf_exporter.
And include that urdf file and sia20's urdf into new one file.

#### Check list

- [ ] link
- [ ] joint
- [ ] transmission
- [ ] gazebo **(especially robotNamespace)**

Attach your gripper link to link ```tool0```.

###  3. Verify your gripper in rviz

In order to look your gripper and sia20 in rviz, you can use existing launch file.

```sh
roslaunch sia20_description display model:=`$(find your_package)/urdf/sia20_with_gripper.urdf)'
```

###  4. Make configuration yaml file

Add configuration **YAML** file in config directory.

That configuration file tells ```ros parameter server``` some configuration as below. And ```ros_controller(pid controller or jont_state_controller)``` use such configuration.

- [Output] which joint information will be publish from ```ros_controller node```.
- [Input] which joint use which control way(position, velocity, or effort) and pid gain in ```ros_controller node```

Those configuration mush much with configuration in urdf or xacro that you wrote.



### 5. Make controller launch file

In order to attach pid controller to each joint and receive joint state(angle, velocity, and accelaletion),

set configuration in YAML file to ```ros parameter server``` using launch file.

- [ ] Load joint controller configuration from YAML file to parameter server
- [ ] Spawn pid controllers using configuration
- [ ] Spawn robot_state_publisher to convert joint state to tf transforms for rviz, etc

### 6. Make gazebo launch file

- [ ] load empty gazebo world
- [ ] load robot_description from urdf file and set to ```ros parameter server```
- [ ] spawn robot model using urdf file.



### 7. launch in gazebo

### 8. make moveit configuration

