#!/bin/sh
trap "kill 0" 2

#counttr=`ps aux | grep torch | grep -v grep | grep -v vim | wc -l`
#if [ "$counttr" != "0" ]; then
#  ps aux | grep torch | grep -v grep | xargs sh -c 'kill -9 $1'
#fi


#roslaunch ur5_gripper_moveit_config gazebo.launch &
roslaunch ur5_gripper_moveit_config gazebo.launch gazebo_gui:=False &
world_pid=$!
sleep 10

checkgz=`ps -ef | grep gzserver | grep -v grep | wc -l`
if [ "$checkgz" = "0" ]; then
  kill $world_pid
  wait $world_pid
  roslaunch ur5_gripper_moveit_config gazebo.launch gazebo_gui:=False &
  world_pid=$!
  sleep 10
fi

sleep 10

#gz physics -u 0
gz physics -s 0.003
roslaunch ur5_gripper_moveit_config move_group.launch &
move_pid=$!
sleep 8

python ur_control.py -r &
ur_pid=$!

python tac_flag.py &
tac_pid=$!


sleep 8

cc=0
#cc=`expr $cc + 1`
until [ $cc -eq 18000 ]
do
  sleep 10
  countur=`ps -aux | grep ur_control | grep -v grep | grep -v vim |wc -l`
  countgz=`ps -ef | grep gzserver | grep -v grep | wc -l`
  countmv=`ps -aux | grep move_group.launch | grep -v grep | wc -l`
  counttac=`ps -aux | grep tac_flag.py | grep -v grep | grep -v vim | wc -l`
  if [ "$countur" = "0" ] || [ "$countgz" = "0" ] || [ "$countmv" = "0" ] || [ "$counttac" = "0" ]; then
    break
  fi
done

echo "loop break!!"
kill $tac_pid
kill $ur_pid
kill $move_pid
kill $world_pid

sleep 10
wait $world_pid
echo "loop finish"


