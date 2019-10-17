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
 *  file  : usb_cam_dev.h
 *  brief : the usb camera (UVC) based on the interface of opencv
 *  author: fwc
 *  date  : 2019-2-20
 ***************************************************************************/
#ifndef ROBOT_CAM_USB_CAM_DEV_H
#define ROBOT_CAM_USB_CAM_DEV_H
#include "robot_cam/cam_dev_interface.h"
#include <opencv2/opencv.hpp>

namespace robot_cam {
    class UsbCamDev : public CamDevInterface{
    public:
        UsbCamDev();
        ~UsbCamDev();
    public:
        int init(std::string config_path);
        bool isOpened();
        int capImg(cv::Mat &img);
        
        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);
        bool setParameters(std::string config_path);

    private:
        int openCap();
        bool setExposure(int value);
    private:
        std::string dev_path_;
        cv::VideoCapture cap_;
        int cam_height_;
        int cam_width_;
        int cam_fps_;
        //flag
        bool is_init_;
        bool is_open_;
    };
}

#endif //ROBOT_CAM_USB_CAM_DEV_H



