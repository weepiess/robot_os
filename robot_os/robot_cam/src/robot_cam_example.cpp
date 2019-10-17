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
 *  file  : robot_cam_example.cpp
 *  brief : a general ros node class example for robot cam.
 *  author: fwc
 *  date  : 2019-1-30
 ***************************************************************************/
#include "robot_cam/robot_cam_example.h"
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace robot_cam;

RobotCamExample::RobotCamExample() { run_flag_ = false; }

RobotCamExample::~RobotCamExample() {}

int RobotCamExample::init(CamDevInterface *cam_intercace,std::string config_path) {
    cam_intercace_ = cam_intercace;
    //load parameters
    FileStorage f(config_path, FileStorage::READ);
    if (!f.isOpened()) {
        cerr << "Failed to open " << config_path << endl;
        return -2;
    }
    f["cam_topic_name"] >> cam_name_;
    f.release();
    //set fps
    int fps;
    if(cam_intercace_->getParameter(Fps,fps)){
        fps_period_ms_ = 1000.0 / fps;
    }else{
        fps_period_ms_=1;
    }
    nh_ = ros::NodeHandle(cam_name_);
    // camera info
    cam_info_ = boost::make_shared<sensor_msgs::CameraInfo>();
    cam_info_->header.frame_id = cam_name_;
    // init ros node,and advertise the main image topic
    image_transport::ImageTransport it(nh_);
    img_pub_ = it.advertiseCamera("image_raw", 1, true);
    // start the camera
    int ret=-1;
    // print info
    if (cam_intercace_->isOpened()) {
        ROS_INFO("[cam node]:cam node start ok!");
        ret=0;
    } else {
        ROS_INFO("[cam node]:ERROR,cam node start failed!");
        // ROS_ERR("ERROR,DH cam node start failed!,try to restart...");
    }
    // start cam thread
    cam_thread_ = std::thread(&RobotCamExample::capThread, this);
    run_flag_ = true;
    return ret;
}

void RobotCamExample::capThread() {
    cv::Mat img;
    int ret;
    while (ros::ok()) {
        if (run_flag_) {
            cap_start_ = ros::Time::now();
            ret = cam_intercace_->capImg(img);
            if (ret == 0) {
                sensor_msgs::ImagePtr img_msg =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img)
                        .toImageMsg();
                img_msg->header.frame_id = cam_name_;
                img_msg->header.stamp = cap_start_;
                cam_info_->header.stamp = img_msg->header.stamp;
                img_pub_.publish(img_msg, cam_info_);
                fps_cap_ms_ =
                    (ros::Time::now().toSec() - cap_start_.toSec()) * 1000;
                double time_diff = fps_period_ms_ - fps_cap_ms_;
                // fps control
                if (time_diff > 1000) {  //>1000ms
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds((int)time_diff));
                } else if (time_diff > 0.1) {  // 0.1-1000ms
                    std::this_thread::sleep_for(std::chrono::microseconds(
                        (int)((time_diff - 0.1) * 1000)));
                } else {
                    // pass
                }
            } else {
                ROS_INFO("[cam node]:cap err!");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            //休眠挂起
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    ROS_INFO("[cam node]:cap thread exit!");
}

