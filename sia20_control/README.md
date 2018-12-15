# sia20_control package

## launch 
### sia20_control.launch
```sia20_control.launch``` is a launch file for spawning 7 controllers to attach to each joints.
- sia20_control.launch
	- [param server] sia20_control/config/sia20_control.yaml
	- [node] controller_spawner
		- joint_s_position_controller
		- joint_l_position_controller
		- joint_e_position_controller
		- joint_u_position_controller
		- joint_r_position_controller
		- joint_b_position_controller
		- joint_t_position_controller
		- joint_state_controller
	- [node] robot_state_publiser ( joint angle [tf]) /joint_states -> /sia20/joint_states
