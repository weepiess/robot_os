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
 *  file  : sim_cam_image_dev.h
 *  brief : the simulated camera with image based on the interface of opencv
 *  author: fwc
 *  date  : 2019-2-20
 ***************************************************************************/
#ifndef ROBOT_CAM_SIM_CAM_IMAGE_DEV_H
#define ROBOT_CAM_SIM_CAM_IMAGE_DEV_H
#include "robot_cam/cam_dev_interface.h"
#include <opencv2/opencv.hpp>


namespace robot_cam {
    class SimCamImageDev : public CamDevInterface{
    public:
        SimCamImageDev();
        ~SimCamImageDev();
    public:
        void setImgRootPath(std::string root);
        int init(std::string config_path);
        bool isOpened();
        int capImg(cv::Mat &img);

        bool setParameter(CamParameter parameter,int value);
        bool getParameter(CamParameter parameter,int& value);
        bool setParameters(std::string config_path);

    private:
        std::string root_;

        int temp = 0;

        cv::Mat img_; //img cap buffer 
        //resolution
        int cam_width_;
        int cam_height_;
        //flag
        bool is_open_;
        //para
        int cam_fps_;
        
    };
}

#endif //ROBOT_CAM_SIM_CAM_IMAGE_DEV_H



