/****************************************************************************
 *  Copyright (C) 2019 UESTC RoboMaster .
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
 *  file  : usb_cam_thread.h
 *  brief : usb cam capture by a thread
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-05-23
 ***************************************************************************/
#ifndef ROBOT_CAM_USB_CAM_THREAD_H
#define ROBOT_CAM_USB_CAM_THREAD_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include "robot_cam/cam_dev_interface.h"

namespace robot_cam_uestc {
class RobotCamThread {
   public:
    RobotCamThread();
    ~RobotCamThread();

   public:
    int init(robot_cam::CamDevInterface *cam_intercace,std::string config_path);
    int capImg(cv::Mat &img);

    bool setResolution(int width,int height);
    //set and get a parameter
    bool setParameter(robot_cam::CamParameter parameter,int value);
    bool getParameter(robot_cam::CamParameter parameter,int &value);
    //set some parameters by config file
    bool setParameters(std::string config_path);


    bool isOpened();
    bool isRunning();
    void setCapThreadFlag(bool run_flag);

   private:
    void capThread();

   private:
    robot_cam::CamDevInterface* cam_intercace_; // camera_device 
    bool run_flag_;
    std::thread cam_thread_;
    double fps_period_ms_;

    std::mutex img_mutex_; //image capture thread mutex
    cv::Mat img_buf_[30]; 
    bool is_img_update_;
    int img_count_;
    int cam_width_;
    int cam_height_;

    bool update_resolution_flag_=false;
};
}  // namespace robot_cam

#endif  // ROBOT_CAM_USB_CAM_THREAD_H
