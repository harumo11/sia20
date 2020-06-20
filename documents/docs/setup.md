# Setup

This documents describes how to move a sia20 robot with on-the-fly(velocity) control.

This document is considered following environment.

|OS|Ubuntu18.04|
|--|-----------|
|ROS dist.| Melodic|

## Dependency

### Basic

|LINK|EXPLANATION|
|------|----|
|ROS Melodic|ROS|
|[sia20-point-streaming](https://github.com/harumo11/sia20-point-streaming)|ROS package for controlling of the on-the-fly(velocity)|
|[pose-following](https://github.com/harumo11/manipulator_pose_following)|ROS package for calculating inverse kinematics|

### Optional

|LINK|EXPLANATION|
|------|----|
|[sia20](https://github.com/harumo11/sia20)|Some utility programs for using a sia20|

## 使い方

### 1. キーボードで動かす
これは[pose-following](https://github.com/harumo11/manipulator_pose_following)の機能の一部です．下記のキーを押すことでロボットを操作することができます．一定速度で動かしたいときに便利です．この方法だとロボットの応答に遅れを感じるかもしれません．

| Key   | 説明              |
| ----- | ----------------- |
| 1(q)  | +x(-x)            |
| 2(w)  | +y(-y)            |
| 3(e)  | +z(-z)            |
| 4(r)  | +roll(-roll)      |
| 5(t)  | +pitch(-pitch)    |
| 6(y)  | +yaw(-yaw)        |
| 0(-p) | velocity up(down) |

```Bash
roslaunch motoman_sia20d_moveit_config moveit_planning_execution.launch sim:=false controller:=fs100 robot_ip:=10.0.0.2
```
```Bash
rosservice call /robot_enable
```
```Bash
roslaunch manipulator_pose_following pose_following.launch
```
```Bash
rosservice call /pose_following/start
```
```Bash
roslaunch manipulator_pose_following kb_jogging.launch
```

### 2. DualShock4で動かす

これは[sia20](https://github.com/harumo11/sia20)の機能の一部です．PlayStation4(Sony)のDualShock4を利用することで手先の速度制御を行うことができます．上記のキーボードで動かす方法よりロボットの遅れは少なく感じられるはずです．速度を色々変えて試してみたいときに便利です．
下記のコマンドを入力する前にDualShock4をUSB経由でPCに接続してください．
ただし，安全のために移動速度はゆっくりにしてあります．

SIA20のコントロール方法は下記のとおりです．
 
|方向|ボタン|
|----|------|
|左アナログスティック(上下)|x方向|
|左アナログスティック(左右)|y方向|
|左アナログスティック(上下)+☓ボタン|z方向|
|右アナログスティック(上下)|roll方向|
|右アナログスティック(左右)|pitch方向|
|R2, L2|yaw方向|

```Bash
roslaunch motoman_sia20d_moveit_config moveit_planning_execution.launch sim:=false controller:=fs100 robot_ip:=10.0.0.2
```

```Bash
rosservice call /robot_enable
```

```Bash
roslaunch manipulator_pose_following pose_following.launch
```

```
rosservice call /pose_following/start
```

```Bash
rosrun joy joy_node
```

```Bash
rosrun sia20_control ds4totwist
```


### 3. HTC Viveで動かす

!!! note
	現在作成中です．


### 4. プログラムから使う

**pose_following/cmd_vel**という名前をつけて
**geometry/Twist**型のトピックをpublishすると手先速度を操作することができます．

[サンプルプログラム](https://github.com/harumo11/sia20/blob/master/sia20_control/src/ds4totwist.cpp)

## 理論

下記の計算式を解いて手先速度から関節角度を導いています．

$$ \boldsymbol{J}^{+} = \boldsymbol{J}^{T}(\boldsymbol{J} \boldsymbol{J}^T)^{-1}$$

$$ \boldsymbol{\dot q} = \boldsymbol{J}^{+} \boldsymbol{\dot r}$$

ここで$\boldsymbol{J}^{+}$はヤコビ行列の疑似逆行列であり，$\boldsymbol{\dot q}$は関節速度ベクトル（次元は7)，$\boldsymbol{\dot r}$は手先の速度ベクトルです(次元は６)．
全関節の動きをなるべく抑えるという拘束条件のもと，速度の逆運動学を計算しています．そのため，いきなり大きな動きをするということはありません．

## トラブルシューティング

!!! note
	現在作成中です．
