/****************************************************************************
 *  Copyright (C) 2019 RoboMasterOS.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : task_show_image_node.h
 *  brief : 图像显示节点（开发样例）
 *  author: FWC
 *  date  : 2018-12-13
 ***************************************************************************/
#include "ros/ros.h"
#include "task_image_related/task_show_image.h"
#include <signal.h>
#include "ros/package.h"


void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}


int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "task_image_related_node",ros::init_options::NoSigintHandler);
  std::string filename = ros::package::getPath("task_image_related") + "/res/task_show_image_config.yaml";
  task_image_related::TaskShowImage task;
  //初始化任务，处理任务开始运行
  task.init(filename);
  //ros spin 调度线程,这里ROS图片订阅需要spin调度。
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ros::waitForShutdown();
  return EXIT_SUCCESS;
}
