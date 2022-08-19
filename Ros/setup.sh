#!/bin/bash
basepath=$(cd `dirname $0`; pwd)
cd $basepath

rm -r ~/fantasy_ws
mkdir -p ~/fantasy_ws/src
cp -r ./auto_driver/ ./bluetooth_bridge/ ./rplidar_ros/ ~/fantasy_ws/src/
chmod +x ~/fantasy_ws/src/auto_driver/src/*.py ~/fantasy_ws/src/bluetooth_bridge/src/*.py ~/fantasy_ws/src/rplidar_ros/src/*.py

cd ~/fantasy_ws
catkin_make
echo "source ~/fantasy_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc