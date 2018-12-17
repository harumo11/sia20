# sia20_moveit_config package

## launch
### planning_context.launch
```planning_context.launch``` is launch file for loading configurations and uploading to parameter server

- planning_context.launch
	- [param server] robot_description <- sia20_description/robots/sia20.urdf.xacro
	- [param server] robot_description_semantic <- sia20_moveit_config/config/sia20.srdf
	- [param server] robot_description_planning <- sia20_moveit_config/config/joint_limits.yaml
	- [param server] robot_description_kinematics <- sia20_moveit_config/config/kinematics.yaml

### move_group.launch
​```move_group.launch``` is launch file for running move_group node
- move_group.launch
	- [include] include sia20_moveit_config/launch/planning_context.launch
	- [other] GDB Debug option (debug:=trueで発動?)
	- [other] verbose mode option (debug:=trueで発動?)
	- [arg] allow_trajectory_execution = ftru
	- [arg] fake_execution = false
	- [arg] max_sefe_path_cost = 1
	- [arg] jiggle_fraction = 0.5
	- [arg] publish_monitored_planning_scene = true
	- [arg] capabilities = 
	- [arg] disable_capabilities =
	- [arg] capabilities = "a_package/AwsomeMotionPlanningCapability another_package/GraspPlanningPipeline"
	- [arg] disable_capabilities = "move_group/MoveGroupKinematicsService move_group/ClearOctmapService"
	- [include] include sia20_moveit_config/launch/planning_pipeline.launch (planning funcitionality)
	- [include] include sia20_moveit_config/launch/trajectory_execution.launch (trajectory execution functionality)
	- [include] include sia20_moveit_config/launch/sensor_manager.launch (sensor functionality)
	- [node] move_group 

### trajectory_execution.launch

- trajectory_execution.launch
	- [param server] moveit_manage_controllers = true (MoveItにcontrollerのロード・アンロード・スイッチを許可する)
	- [param server] trajectory_execution/allowed_execution_duration_scaling = 1.2 (trajectoryの実行にかかる時間にこの係数が掛けられる)
	- [param srever] trajectory_execution/allowed_goal_duration_margin = 0.5 (起動のキャンセルを取りがする前に予想される実行時間を超えることを許可する？)
	- [param server] trajectory_execution/allowed_start_tolerance = 0.01 (計算されたtrajectoryの最初の位置と現在の位置のズレをどれくらい許容するか)
	- [include] sia20_moveit_config/launch/sia20_moveit_controller_manager.launch.xml(moveit_controller_managerのパラメータ)

### planning_execution.launch
-planning_execution.launch
	- [include] include sia20_moveit_config/launch/ompl_pipeline_pipeline.launch.xml (ここで読み込むファイルを変更すると，異なるプランニングパイプラインが使用できる)


