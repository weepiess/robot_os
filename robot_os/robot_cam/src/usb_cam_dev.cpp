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
 *  file  : usb_cam_dev.cpp
 *  brief : the usb camera (UVC) based on the interface of opencv
 *  author: fwc
 *  date  : 2019-2-20
 ***************************************************************************/
#include "robot_cam/usb_cam_dev.h"
#include <thread>

using namespace cv;
using namespace std;
using namespace robot_cam;

UsbCamDev::UsbCamDev() {
    is_init_ = false;
    is_open_ = false;
}

UsbCamDev::~UsbCamDev() {}

int UsbCamDev::init(std::string config_path) {
    FileStorage f(config_path, FileStorage::READ);
    if (!f.isOpened()) {
        cerr << "Failed to open " << config_path << endl;
        return -2;
    }
    f["usb_cam_path"] >> dev_path_;
    f.release();
    //set config path
    setParameters(config_path);
    is_init_ = true;
    if (openCap() == 0){
        is_open_ = true;
        return 0;
    }
    return -1;
}

int UsbCamDev::openCap() {
    if (!is_init_) {
        return -1;
    }
    /***open device*****/
    if (cap_.open(dev_path_)) {
        cap_.set(cv::CAP_PROP_FOURCC,
                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
        is_open_ = true;
        return 0;
    } else {
        return -2;
    }
}

bool UsbCamDev::isOpened() { return is_open_; }

int UsbCamDev::capImg(cv::Mat &img) {
    int ret = -1;
    if (cap_.isOpened()) {
        if (cap_.read(img)) {
            ret = 0;
        } else {
            ret = -2;
        }
    } else {
        ret = -1;
    }
    if (ret != 0) {
        cout << "cap-" << dev_path_ << "-error,reconnecting!" << endl;
        //重连摄像头
        is_open_ = false;
        if (cap_.isOpened()) {
            cap_.release();
        }
        if (openCap() != 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(500));  // 500ms
        }
    }
    return ret;
}

bool UsbCamDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case ResolutionWidth:
            cam_width_=value;
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
            return true;
        case ResolutionHeight:
            cam_height_=value;
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
            return true;   
        case Exposure:
            return setExposure(value);
        case Fps:
            cam_fps_=value;
            return true; 
        default:
            return false;

    }
}

bool UsbCamDev::getParameter(CamParameter parameter, int& value) {
    switch (parameter) {
        case ResolutionWidth:
            value=cam_width_;
            return true;
        case ResolutionHeight:
            value=cam_height_;
            return true;    
        case Exposure:
            return false;
        case Fps:
            value = cam_fps_;
            return true;
        default:
            return false;
    }
}


bool UsbCamDev::setParameters(std::string config_path) {
    FileStorage f(config_path, FileStorage::READ);
    if (!f.isOpened()) {
        cerr << "Failed to open " << config_path << endl;
        return false;
    }
    //Resolution
    f["cam_width"] >> cam_width_;
    f["cam_height"] >> cam_height_;
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
    //Fps
    f["cam_fps"] >> cam_fps_;
    //Exposure
    int value;
    f["cam_exposure"] >> value;
    setExposure(value);
    f.release();
    return true;
}

bool UsbCamDev::setExposure(int value){
    //TODO:setting of exposure isn't supported in OpenCV
    return false;
}
