#!/bin/bash
# 启动多点导航
gnome-terminal \
--tab -e "zsh -c 'source $HOME/ros_ws/.zshrc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch hiwonder_navigation navigation.launch map:=explore robot_name:=/ master_name:=/ & sleep 10;roslaunch hiwonder_navigation publish_point.launch enable_navigation:=false robot_name:=/ master_name:=/ & rviz rviz -d  $HOME/ros_ws/src/hiwonder_navigation/rviz/navigation_desktop.rviz'"
