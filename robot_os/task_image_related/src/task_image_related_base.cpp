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
 *  file  : task_image_related_base.h
 *  brief : image process task base class
 *  author: FWC
 *  date  : 2019-04-02
 ****************************************************************************/

#include "task_image_related/task_image_related_base.h"
#include <cv_bridge/cv_bridge.h>
#include <signal.h>

using namespace cv;
using namespace std;
using namespace task_image_related;

TaskImageRelatedBase::TaskImageRelatedBase()
{
    run_flag_=false;
    initflag_=false;
    get_img_flag_=false;
}


int TaskImageRelatedBase::init(std::string conf_path){
    string cam_topicname;
    FileStorage f(conf_path, FileStorage::READ);
    if (!f.isOpened()){
        cerr << "Failed to open " << conf_path << endl;
        return -1;
    }
    f["cam_topic_name"] >> cam_topicname;
	f.release();
    //create subscriber,subscribe image topic
    image_transport::ImageTransport it(nh_);
    cam_sub_ = it.subscribeCamera(cam_topicname, 1,boost::bind(&TaskImageRelatedBase::imgSubCallBack, this, _1,_2));
    /*************task init***************************/
    if(initTask(conf_path)!=0){
        return -2;
    }
    task_thread_= std::thread(&TaskImageRelatedBase::mainTask, this);
    initflag_ = true;
    return 0;
}


void TaskImageRelatedBase::imgSubCallBack(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
    //update camera info,only once.
    if(!img_info_flag_){
        //armor_detector_->setCameraInfo(*camera_info_msg);
        img_info_flag_=true;
    }
    if(run_flag_){
       imgbuf_=cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
       if(!get_img_flag_){
          img_=imgbuf_.clone();
          img_stamp_=img_msg->header.stamp.toSec();
          get_img_flag_=true;
       }
    }
}

void TaskImageRelatedBase::mainTask() {
  int ret;
  while(ros::ok()) {
    if(run_flag_){
        if(get_img_flag_){//表示获取到图片
           taskImageProcess(img_,img_stamp_);
           get_img_flag_=false;//处理完，置位
        }else{
           taskImageWait();
           std::this_thread::sleep_for( std::chrono::milliseconds(2));
        }
    }else{
        taskSleep();
        std::this_thread::sleep_for( std::chrono::milliseconds(100));//休眠挂起
    }  
  }
}

void TaskImageRelatedBase::setRunFlag(bool flag){
    run_flag_=flag;
}

bool TaskImageRelatedBase::isInitOK(){
    return initflag_;
}


