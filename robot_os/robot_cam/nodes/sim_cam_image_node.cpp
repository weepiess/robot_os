/****************************************************************************
 *  Copyright (C) 2019 RobotOS.
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
 *  file  : image_sim_cam_node.h
 *  brief : 图片模拟相机节点
 *  author: fwc
 *  date  : 2019-1-31
 ***************************************************************************/
#include "ros/ros.h"
#include "robot_cam/sim_cam_image_dev.h"
#include "robot_cam/robot_cam_example.h"
#include "ros/package.h"
#include <signal.h>
#include <opencv2/opencv.hpp>


void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}


int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "robot_cam_node",ros::init_options::NoSigintHandler);
  //config path
  std::string config_path = ros::package::getPath("robot_cam") + "/res/sim_cam_image_config.yaml";
  //cam dev
  robot_cam::SimCamImageDev sim_cam;
  sim_cam.setImgRootPath(ros::package::getPath("robot_cam"));
  sim_cam.init(config_path);
  //cam node
  robot_cam::RobotCamExample cam_example;
  cam_example.init(&sim_cam,config_path);
  
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ros::waitForShutdown();
  return EXIT_SUCCESS;
}
