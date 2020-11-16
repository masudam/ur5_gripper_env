# ur5_gripper_moveit_config

ur5とgripperを付けた環境

https://github.com/utecrobotics/ur5
を元にmoveit_setup_Assitantなどで作成

このgitと
https://github.com/utecrobotics/robotiq.git
をwork_spaceにcloneして使う


### 実行
```
roslaunch ur5_gripper_moveit_config gazebo.launch
```
moveitなどを起動する場合は追加で
```
roslaunch ur5_gripper_moveit_config moveit_rviz.launch
```
