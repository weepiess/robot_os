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
 *  file  : task_show_image.h
 *  brief : 显示图像任务
 *  author: FWC
 *  date  : 2019-2-13
 ***************************************************************************/
#ifndef TASK_IMAGE_RELATED_TASK_SHOW_IMAGE_H
#define TASK_IMAGE_RELATED_TASK_SHOW_IMAGE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "task_image_related/task_image_related_base.h"

namespace task_image_related {
   class TaskShowImage:public TaskImageRelatedBase
   {
   public:
     TaskShowImage();
     ~TaskShowImage();
   private:
     int initTask(std::string conf_path);
     void taskImageProcess(cv::Mat& img,double img_stamp);
     void taskImageWait();
     void taskSleep();
  };
}

#endif //TASK_IMAGE_RELATED_TASK_IMAGE_RELATED_BASE_H




