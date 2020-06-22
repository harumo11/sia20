# HTC Vive 

ROSでHTC　Viveを使用するための説明

## Dependency

- [ROS Package](https://github.com/robosavvy/vive_ros)
- [SteamVR](https://store.steampowered.com/steamvr?l=japanese)

## ROS

Viveを立ち上げてROSで使用するときには下記のコマンドを実行します．

1. launch vive server

```
roslaunch vive_ros server_vr.launch
```

2.  launch ROS topic bridge

```
rosrun vive_ros vive.launch
```

3. rvizで確認

```
rviz
```

Fixed Frame -> world
Add -> By Display Type -> TF

## 座標系

本棚の上のベースステーションが{world}の上に存在する．
ただ床から2.265m上空だと表示されるが，実際は2.1mなので誤差がある．
steam-vrのroom scaleを再度実行して調整すると元に戻るかも．．


## トラブルシューティング

### roslaunchでエラーが起きてプロセスが死ぬ

1. 以前に使用したviveのサーバプログラムがゾンビプロセスになっている可能性があります．

   `rosrun vive_ros close_server.sh`を実行することによって多くの場合はゾンビプロセスをkillすることができます．killした後，もう一度roslaunchを試みてください．

### steamVRでfaildというエラーが出る．
ROSからはアクセス可能な場合があります．ROSからアクセスできるか試してみてください．

### NVidia cudaのドライバーのアップデートにより，steamが起動できなくなる．

1. Software & Updates -> Additional Driversのドライバーがオープソースのものが選択されている可能性がある．一番最新の**proprietary**のドライバーを選択しよう．
その後`sudo apt update && sudo apt install cuda`すると治るかもしれない．

### Streamは起動するがStream-VRをしようとすると，VIVEをPCに挿してくださいと出る．当然PCにVIVEは繋がっている．
udevのruleはSteam-VRのインストールと同時にインストールされるが，失敗したと思われる．
[ここ](https://github.com/ValveSoftware/steam-devices/blob/master/60-steam-vr.rules)をダウンロードして
/etc/udev/rules.d/60-steam-vr.rulesに入れましょう．
その後，`  sudo udevadm control --reload-rules && sudo udevadm trigger`を実行し，steamを立ち上げ確認する．
