# Cartesian Velocity Control

直交座標系での手先の速度コントロールのためのパッケージ作成プロジェクト

!!! quote
	- [rqt_joint_trajectory_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/rqt_joint_trajectory_controller)
	- [motoman_control_node](https://github.com/Nishida-Lab/motoman_project/blob/indigo-devel/motoman_control/src/motoman_control_node.cpp)
	

## SIA20の制御メッセージの流れ

```mermaid
graph TD;
MoveIt! -- Follow Joint Trajectory <br> ACTIONLIB MESSAGE --> motoman_control_node;
motoman_control_node -- JointTrajectory <br> TOPIC MESSAGE --> motoman_driver;
motoman_driver -- simple message <br> ROS-I PROTCOL --> SIA20;
```

`rqt_joint_trajectory_controller`というrqtのプラグインを入れることによって
`JointTrajectory`のトピック(各関節ごとの値)をrqtから送ることができる．
これと同様のプログラムをC++で作成することから始め用と思う．

このプログラムは`sia20_control`に入れておこうと思う．

## やること

1. rqt_joint_trajectory_controllerが内部で何をやっているのかを調べる
2. nishida_labの`motoman_control_node`がどのように'JointTrajectory`メッセージを作成しているのかを調べる
2. C++でパッケージを作成

