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
 *  GNU General Public Licens  e for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : robot_cam_example.h
 *  brief : a general ros node class example for robot cam.
 *  author: fwc
 *  date  : 2019-1-30
 ***************************************************************************/
#ifndef ROBOT_CAM_ROBOT_CAM_EXAMPLE_H
#define ROBOT_CAM_ROBOT_CAM_EXAMPLE_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Empty.h>
#include <thread>
#include "robot_cam/cam_dev_interface.h"

namespace robot_cam {
   class RobotCamExample
   {
   public:
     RobotCamExample();
     ~RobotCamExample();
   public:
     int init(CamDevInterface *cam_intercace,std::string config_path);
   private:
     void capThread(); 

   private:
     ros::NodeHandle nh_;  // private ROS node handle
     sensor_msgs::CameraInfoPtr cam_info_;
     image_transport::CameraPublisher img_pub_;
     ros::ServiceServer srv_start_, srv_stop_;
     CamDevInterface *cam_intercace_;   // camera_device interface
     std::string cam_name_;
     bool run_flag_;
     std::thread cam_thread_;
     ros::Time cap_start_;
     double fps_cap_ms_;
     double fps_period_ms_;
  };
}

#endif //ROBOT_CAM_ROBOT_CAM_EXAMPLE_H



