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
 *  file  : robot_cam_thread.h
 *  brief : robot cam capture by a thread
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-5-23
 ***************************************************************************/
#include "robot_cam_uestc/robot_cam_thread.h"

using namespace cv;
using namespace std;
using namespace robot_cam;
using namespace robot_cam_uestc;

RobotCamThread::RobotCamThread() { run_flag_ = false; }

RobotCamThread::~RobotCamThread() {}

int RobotCamThread::init(CamDevInterface *cam_intercace,
                         std::string config_path) {
    cam_intercace_ = cam_intercace;
    // fps control
    int fps;
    if (cam_intercace_->getParameter(Fps, fps)) {
        fps_period_ms_ = 1000.0 / fps;
    } else {
        fps_period_ms_ = 1;
    }
    // get resolution
    if (!(cam_intercace_->getParameter(ResolutionWidth, cam_width_) &&
          cam_intercace_->getParameter(ResolutionHeight, cam_height_))) {
        cerr << "[cam thread]:init e rr,get resolution err!" << endl;
        return -1;
    }
    // check the camera
    int ret = -1;
    if (cam_intercace_->isOpened()) {
        cout << "[cam thread]:cam node start ok!" << endl;
        ret = 0;
    } else {
        cerr << "[cam thread]:init err,cam node start failed!" << endl;
    }
    // start cam thread
    cam_thread_ = std::thread(&RobotCamThread::capThread, this);
    is_img_update_ = false;
    run_flag_ = true;
    return ret;
}

// copy img from img buffer of cap_thread
int RobotCamThread::capImg(cv::Mat &img) {
    if (!is_img_update_) {
        //等待100ms
        int timeCounter = 0;
        while (!is_img_update_ && timeCounter < 100) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));  // sleep 1ms
            timeCounter++;
        }
        if (!is_img_update_) {
            return -3;  // update timeout
        }
    }
    //img_mutex_.lock();     // thread lock
    img = img_buf_[img_count_];  // copy img from buffer
    //img_mutex_.unlock();   // thread unlock
    if (!img.empty() && (img.cols == cam_width_) && (img.rows == cam_height_)) {
        is_img_update_ = false;
        return 0;
    } else {
        return -1;
    }
}

void RobotCamThread::capThread() {
    int ret;
    while (true) {
        if (update_resolution_flag_) {
            cam_intercace_->setParameter(ResolutionWidth, cam_width_);
            cam_intercace_->setParameter(ResolutionHeight, cam_height_);
            update_resolution_flag_ = false;
        }
        if (run_flag_) {
            auto cap_start = chrono::system_clock::now();
            ret = cam_intercace_->capImg(img_buf_[img_count_]);
            img_count_++;
            if(img_count_==30)
                img_count_=0;
            if (ret == 0) {
                is_img_update_ = true;
                auto cap_end = chrono::system_clock::now();
                double fps_cap_ms =
                    chrono::duration_cast<chrono::duration<double> >(cap_end -
                                                                     cap_start)
                        .count() *
                    1000;
                double time_diff = fps_period_ms_ - fps_cap_ms;
                // fps control
                // if (time_diff > 100) {  //>1000ms
                //     std::this_thread::sleep_for(
                //         std::chrono::milliseconds((int)time_diff));
                // } else if (time_diff > 0.1) {  // 0.1-100ms
                //     std::this_thread::sleep_for(std::chrono::microseconds(
                //         (int)((time_diff - 0.1) * 1000)));
                // } else {
                //     // pass
                // }
            } else {
                cout << "[cam node]:cap err!" << endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            //休眠挂起
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    cout << "[cam node]:cap thread exit!" << endl;
}

bool RobotCamThread::isOpened() { return cam_intercace_->isOpened(); }

bool RobotCamThread::isRunning() { return run_flag_; }

void RobotCamThread::setCapThreadFlag(bool run_flag) { run_flag_ = run_flag; }

bool RobotCamThread::setParameter(CamParameter parameter, int value) {
    if (parameter == ResolutionWidth || parameter == ResolutionHeight) {
        cout << "in cam thread,use func setResolution to set resolution "
                "instead of setParameter"
             << endl;
        return false;
    }else if(parameter == Fps) {
        // update fps
        fps_period_ms_ = 1000.0 / value;
    }else{}
    return cam_intercace_->setParameter(parameter, value);
}
bool RobotCamThread::getParameter(CamParameter parameter,int &value){
    return cam_intercace_->getParameter(parameter,value);
}
bool RobotCamThread::setParameters(std::string config_path) {
    bool flag = cam_intercace_->setParameters(config_path);
    // update fps
    int fps;
    if (cam_intercace_->getParameter(Fps, fps)) {
        fps_period_ms_ = 1000.0 / fps;
    }
    return flag;
}

bool RobotCamThread::setResolution(int width, int height) {
    cam_width_ = width;
    cam_height_ = height;
    update_resolution_flag_ = true;
}