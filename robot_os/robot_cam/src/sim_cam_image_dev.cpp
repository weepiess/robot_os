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
 *  file  : sim_cam_image_dev.cpp
 *  brief : the simulated camera with image based on the interface of opencv
 *  author: fwc
 *  date  : 2019-1-31
 ***************************************************************************/
#include "robot_cam/sim_cam_image_dev.h"
#include <thread>

using namespace robot_cam;
using namespace cv;
using namespace std;

SimCamImageDev::SimCamImageDev() { is_open_ = false; }

SimCamImageDev::~SimCamImageDev() {}

void SimCamImageDev::setImgRootPath(std::string root){
    root_=root;
}

int SimCamImageDev::init(std::string config_path) {
    if (is_open_) {
        return 0;
    } else {
        std::string img_path;
        cv::FileStorage f(config_path, cv::FileStorage::READ);
        if (!f.isOpened()){
            std::cerr << "Failed to open " << config_path << std::endl;
            return -1;
        }
        f["image_path"] >> img_path;
	    f.release();
        //relative path
        if(img_path.at(0)!='/'&&(!root_.empty())){
            img_path=root_+"/"+img_path;
        }
        img_ = imread(img_path);
        if (img_.empty()) {
            cout << "image sim cam error,please check the path " << img_path
                 << "!" << endl;
            return -1;
        } else {
            setParameters(config_path);
            cam_width_=img_.cols;
            cam_height_=img_.rows;
            is_open_ = true;
            return 0;
        }
    }
    return -2;
}

bool SimCamImageDev::isOpened() { return is_open_; }

int SimCamImageDev::capImg(cv::Mat& img) {
    //string img_path = "/home/wyx/图片/5-17-1/";
    if (is_open_) {
        //img_path = img_path + to_string(temp++) + ".jpg";
        //std::cout<<img_path<<std::endl;
        //img = imread(img_path);
        img = img_.clone();
        return 0;
    } else {
        cout << "image sim cam -error!" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return -1;
    }
}

bool SimCamImageDev::setParameter(CamParameter parameter, int value) {
    switch (parameter) {
        case Fps:
            cam_fps_ = value;
            return true;
        default:
            return false;
    }
}

bool SimCamImageDev::getParameter(CamParameter parameter, int& value) {
    switch (parameter) {
        case ResolutionWidth:
            value=cam_width_;
            return true;
        case ResolutionHeight:
            value=cam_height_;
            return true;     
        case Fps:
            value = cam_fps_;
            return true;     
        default:
            return false;
    }
}

bool SimCamImageDev::setParameters(std::string config_path) {
    FileStorage f(config_path, FileStorage::READ);
    if (!f.isOpened()) {
        cerr << "Failed to open " << config_path << endl;
        return false;
    }
    //Fps
    f["cam_fps"] >> cam_fps_;
    f.release();
    return true;
}
