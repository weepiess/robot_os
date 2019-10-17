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
 *  file  : cam_dev_interface.h
 *  brief : base class of cam dev
 *  author: fwc
 *  date  : 2019-1-30
 * 
 * history:
 * 2019-01-31:add parameter interface (height,width and fps).
 * 2019-02-20:code refactor,the class should become a interface with pure
 *           virtual function,not a base class.
 * 2019-05-23:1.add get interface of parameter.
 *            1.add parameter interface.(Support Exposure,Brighthness,
 *            WhiteBalance,Gain,Gamma,Contrast,Saturation,Hue,Fps).
 *            2.add config file interface.
 ***************************************************************************/
#ifndef ROBOT_CAM_CAM_DEV_INTERFACE_H
#define ROBOT_CAM_CAM_DEV_INTERFACE_H

#include <opencv2/opencv.hpp>

namespace robot_cam {
    enum CamParameter {ResolutionWidth,ResolutionHeight,Exposure,Brightness,WhiteBalance,Gain,Gamma,Contrast,Saturation,Hue,Fps,ImgReadTime};
    class CamDevInterface{
    public:
        virtual int init(std::string config_path)=0;
        //major interface (required)
        virtual bool isOpened()=0;
        virtual int capImg(cv::Mat &img)=0;
        //optional interface
        //set and get a parameter
        virtual bool setParameter(CamParameter parameter,int value){return false;};
        virtual bool getParameter(CamParameter parameter,int& value){return false;};
        //set some parameters by config file
        virtual bool setParameters(std::string config_path){return false;};

    };
}

#endif //ROBOT_CAM_CAM_DEV_INTERFACE_H



