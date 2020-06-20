# pose_follow_planner & executerの使い方

## 構造と役割

pose_follow_planner ==(target joint)==> pose_follow_executer

- pose_follow_planner

  HTC Viveから命令を受取りmoveitを用いて逆運動学を解く．

  target_joint = current_joint + $\Delta$ T * joint_displacement

- pose_follow_executer

  `pose_follow_planner`で作成したtarget_jointをタイムスタンプをつけてSIA20へ転送する．

  実行周期はrate = 1/ $\Delta$T を使わないとエラー（#4414 スピードオーバー）が発生する．

## 立ち上げ方

roscore

```
roscore 
```

HTC Vive

```shell
roslaunch vive_ros server_vr.launch
roslaunch vive_ros vive.launch
```

motoman

```
roslaunch motoman_sia20d_moveit_config moveit_planning_execution.launch sim:=false controller:=fs100 robot_ip:=10.0.0.2
```

pose_follow_executer

```
rosrun sia20_control pose_follow_executer
```

pose_follow_planner

```
rosrun sia20_control pose_follow_plannner
```

追従を始める前にロボットの手先方向（ｚ軸）とコントローラの尻尾（ｚ軸）の方向を合わせてからViveのコントローラの丸ボタン押すと直感的に制御しやすい．

## Issue

