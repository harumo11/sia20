# sia20_description package

## launch
### display.launch

```display.launch``` is launch file for verificating robot xacro model.

- display.launch
	- [param server] robot_description <- xacro sia20_description/robots/sia20.urdf.xacro
	- [node] joint_state_publisher (joint angle [radian])
	- [node] robot_state_publisher (joint angle [tf])
	- [node] rviz

### gazebo.launch
```gazebo.launch``` is  launch file for verificating robot model in gazebo.
- gazebo.launch
	- [include] include gazebo_ros/launch/empty_world.launch
	- [param server] robot_description <- xacro sia20_description/robots/sia20.urdf.xacro
	- [node] spawn_model <- model in gazebo
