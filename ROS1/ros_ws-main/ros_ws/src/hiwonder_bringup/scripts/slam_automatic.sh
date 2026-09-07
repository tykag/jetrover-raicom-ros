#!/bin/bash
# 启动explore自主建图
gnome-terminal \
--tab -e "zsh -c 'source $HOME/ros_ws/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch hiwonder_slam slam.launch slam_methods:=explore robot_name:=/ master_name:=/ & sleep 10;rviz rviz -d  $HOME/ros_ws/src/hiwonder_slam/rviz/explore_desktop.rviz'"
