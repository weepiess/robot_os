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
 *  file  : video_sim_cam_interface.h
 *  brief : 基于opencv接口的视频模拟相机接口
 *  author: fwc
 *  date  : 2019-1-30
 ***************************************************************************/
#ifndef ROBOT_CAM_SIM_CAM_VIDEO_DEV_H
#define ROBOT_CAM_SIM_CAM_VIDEO_DEV_H
#include "robot_cam/cam_dev_interface.h"
#include <opencv2/opencv.hpp>

namespace robot_cam {
    class SimCamVideoDev : public CamDevInterface{
    public:
        SimCamVideoDev();
        ~SimCamVideoDev();
    public:
        int init(std::string config_path);
        bool isOpened();
        int capImg(cv::Mat &img);
        
        bool getParameter(CamParameter parameter,int& value);

    private:
        cv::VideoCapture cap_;
        int total_frames_;
        int current_frame;
        //para
        float cam_fps_;
        //resolution
        int cam_width_;
        int cam_height_;
        //flag
        bool is_open_;
    };
}

#endif //ROBOT_CAM_SIM_CAM_VIDEO_DEV_H



