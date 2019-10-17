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
 *  file  : cam_interface_base.h
 *  brief : 相机接口基类
 *  author: fwc
 *  date  : 2019-1-30
 ***************************************************************************/
#include "robot_cam/sim_cam_video_dev.h"
#include <thread>

using namespace cv;
using namespace std;
using namespace robot_cam;

SimCamVideoDev::SimCamVideoDev(){
    current_frame=0;
    is_open_=false;
}

SimCamVideoDev::~SimCamVideoDev(){

}

int SimCamVideoDev::init(std::string config_path){
    std::string video_path;
    FileStorage f(config_path, FileStorage::READ);
    if (!f.isOpened()){
            cerr << "Failed to open " << config_path << endl;
            return -2;
    }
    f["video_path"] >> video_path;
	f.release();
    if(!is_open_){
        if(cap_.open(video_path)){
            cam_height_=cap_.get(CV_CAP_PROP_FRAME_HEIGHT);
            cam_width_=cap_.get(CV_CAP_PROP_FRAME_WIDTH);
            total_frames_=cap_.get(CV_CAP_PROP_FRAME_COUNT);
            cam_fps_ = cap_.get(CV_CAP_PROP_FPS);
            is_open_=true;
            return 0;
        }else{
            cout<<"open err:"<<video_path<<endl; 
        }
    }
    return -1;
    
}


bool SimCamVideoDev::isOpened(){
    return is_open_;
}

int SimCamVideoDev::capImg(cv::Mat &img){
    if(is_open_){
        if(cap_.read(img)){
            current_frame++;
            if(current_frame>total_frames_-2){
                current_frame=0;
                cap_.set(CV_CAP_PROP_POS_FRAMES,0);
            }
            return 0;
        }else{
            return -2;
        }
        return -1;
    }else{
        cout<<"video sim cam -error!"<<endl;
        std::this_thread::sleep_for( std::chrono::milliseconds(500));
        return -1;
    }
    
}



bool SimCamVideoDev::getParameter(CamParameter parameter, int& value) {
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
