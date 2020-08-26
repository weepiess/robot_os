# robot_os
robot_os 简介：
robot_os是基于ROS开发的开源软件栈，包含了开发一个机器人项目的底层功能包，可以方便迅速的快速开始开发。

robot_os包含了5个主要模块：

robot_base
robot_cam
robot_msgs
robot_tool
task_image_related
robot_base 为底层通信模块，暂时只实现了串口通信

robot_cam 为感知模块，暂时只实现了相机的封装

robot_msgs 为基础消息的定义

robot_tool 为常用开发工具模块，封装了PNP,kalman,计时工具,多线程

task_image_related 为图片处理模块
