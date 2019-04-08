# Cartesian Velocity Control

直交座標系での手先の速度コントロールのためのパッケージ作成プロジェクト

!!! quote
	- [rqt_joint_trajectory_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/rqt_joint_trajectory_controller)
	- [motoman_control_node](https://github.com/Nishida-Lab/motoman_project/blob/indigo-devel/motoman_control/src/motoman_control_node.cpp)
	

## SIA20の制御メッセージの流れ

### 実機

```mermaid
graph TD;
MoveIt! -- Follow Joint Trajectory <br> ACTIONLIB MESSAGE --> motoman_control_node;
motoman_control_node -- JointTrajectory <br> TOPIC MESSAGE --> motoman_driver;
motoman_driver -- simple message <br> ROS-I PROTCOL --> SIA20;
```

### シミュレータ

```mermaid
graph TD;
rqt_joint_trajectory_controller -- /sia20/sia20_joint_controller/command <br> TOPIC MESSAGE --> gazebo;
rviz -- /execute_trajectory/action_topics <br> ACTIONLIB MESSAGE --> /move_group;
/move_group -- /sia20/sia20_joint_controller/follow_joint_trajectory/action_topic --> gazebo;
gazebo -- /sia20/joint_states TOPIC MESSAGE --> /move_group;
```

### /sia20/sia20_joint_controller/commandの中身

メッセージタイプは`trajectory_msgs/JointTrajectory`

```
header: 
  seq: 10473
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
joint_names: [joint_b, joint_e, joint_l, joint_r, joint_s, joint_t, joint_u]
points: 
  - 
    positions: [-0.998296, 0.77142, -0.85, -2.56, -3.06, -2.73, -1.84]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 1
      nsecs:         0
```

`rqt_joint_trajectory_controller`というrqtのプラグインを入れることによって
`JointTrajectory`のトピック(各関節ごとの値)をrqtから送ることができる．
これと同様のプログラムをC++で作成することから始め用と思う．

このプログラムは`sia20_control`に入れておこうと思う．

## やること

1. rqt_joint_trajectory_controllerが内部で何をやっているのかを調べる
2. nishida_labの`motoman_control_node`がどのように'JointTrajectory`メッセージを作成しているのかを調べる
2. C++でパッケージを作成

