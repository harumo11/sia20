# 学習実行

## 立ち上げ方
### PC-NGで行うコマンド

roscore
```
roscore
```

motoman
```
roslaunch motoman_sia20d_moveit_config moveit_planning_execution.launch sim:=false controller:=fs100 robot_ip:=10.0.0.2
```

pose_follower
```
rosrun manipulator_pose_following pose_following_jaco_node
```
