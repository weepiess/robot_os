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
 *  file  : task_image_related_base.h
 *  brief : 图像相关任务基类
 *  author: FWC
 *  date  : 2019-2-13
 ***************************************************************************/
#ifndef TASK_IMAGE_RELATED_TASK_IMAGE_RELATED_BASE_H
#define TASK_IMAGE_RELATED_TASK_IMAGE_RELATED_BASE_H

#include <ros/ros.h>
#include <thread>
#include "robot_msgs/SetMode.h"
#include "robot_msgs/GimbalInfo.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

namespace task_image_related {
   class TaskImageRelatedBase
   {
   public:
     TaskImageRelatedBase();
     ~TaskImageRelatedBase(){};
   public:
     int init(std::string conf_path);
   private:
     void mainTask();
     void imgSubCallBack(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &camera_info_msg);
   protected:
     void setRunFlag(bool flag);
     bool isInitOK();
   private:
     ros::NodeHandle nh_;  // private ROS node handle
     bool initflag_;
     std::thread task_thread_;
     cv::Mat imgbuf_,img_;//获取的图片，以及缓存图片
     image_transport::CameraSubscriber cam_sub_;//订阅图片数据
     bool img_info_flag_;//
     bool get_img_flag_;//使用flag实现多线程同步机制
     bool run_flag_; //运行标志位
     double img_stamp_;
    public:
     virtual int initTask(std::string conf_path){return 0;};
     virtual void taskImageProcess(cv::Mat& img,double img_stamp)=0;
     virtual void taskImageWait(){};
     virtual void taskSleep(){};
   
  };
}

#endif //TASK_IMAGE_RELATED_TASK_IMAGE_RELATED_BASE_H




