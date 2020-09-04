# FollowJointTrajectoryAction

- [control_msgs/FollowJointTrajectory.action](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html)
- [trajectory_msgs/JointTrajectory.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)

## FollowJointTrajectory.actionの構造

### ゴール 

```
FollowJointTrajectory.action
	|
	|--- trajectory_msgs/JointTrajectory trajectory
	|
	|--- JointTolerance[] path_tolerance
	|
	|--- JointTolerance[] goal_tolerance
	|
	|--- duration goal_time_tolerance
```

### 結果

```
FollowJointTrajectory.action
	|
	|--- int32 error_code
	|
	|--- int32 SUCCESSFULL = 0
	|
	|--- int32 INVALID_GOAL = -1
	|
	|--- int32 INVALID_JOINTS = -2
	|
	|--- int32 ODL_HEADER_TIMESTAMP = -3
	|
	|--- int32 PATH_TOLERANCE_VIOLATED = -4
	|
	|--- int32 GOAL_TOLERANCE_VIOLATED = -5
	|
	|--- string error_string
```

### フィードバック(途中経過)

```
FollowJointTrajectory.action
	|
	|--- string[] joint_names
	|
	|--- trajectory_msgs/JointTrajectoryPoint desired
	|
	|--- trajectory_msgs/JointTrajectoryPoint actual
	|
	|--- trajectory_msgs/JointTrajectoryPoint error
```

## JointTrajectory.msgの構造

```
JointTrajectory.msg
	|
	|--- std_msgs header
	|		|
	|		|--- uint32 seq (sequential increasing ID)
	|		| 
	|		|--- time stamp (sec and nsec)
	|		|
	|		|--- string frame_id
	|
	|--- string[] joint_names
	|
	|--- trajectory_msgs/JointTrajectoryPoint[] points
			|
			|--- float64[] positions
			|
			|--- float64[] velocities
			|
			|--- float64[] accelerations
			|
			|--- duation time_from_start
```
