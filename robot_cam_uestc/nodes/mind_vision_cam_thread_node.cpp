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
 *  file  : mind_visin_cam_thread_node.cpp
 *  brief : MindVision工业相机模拟节点（多线程获取图像）
 *  author: wyx
 *  email : 1418555317@qq.com
 *  date  : 2019-4-11
 * 
 * history:
 ***************************************************************************/
#include "ros/ros.h"
#include "robot_cam_uestc/mind_vision_cam_dev.h"
#include "robot_cam_uestc/robot_cam_thread.h"

#include <signal.h>
#include "ros/package.h"

using namespace std;
using namespace cv;

void SignalHandler(int signal){
    if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
        ros::shutdown();
    }
}


int main(int argc, char **argv){
    // signal(SIGINT, SignalHandler);
    // signal(SIGTERM,SignalHandler);
    // ros::init(argc, argv, "robot_mindvision_cam_node",ros::init_options::NoSigintHandler);
    std::string config_path = ros::package::getPath("robot_cam_uestc") + "/res/mind_vision_cam_config.yaml";

    //读取主配置文件
    // int fps;
    // cv::FileStorage f(config_path, cv::FileStorage::READ);
    // if (!f.isOpened()){
    //     std::cerr << "Failed to open " << config_path << std::endl;
    //     return EXIT_FAILURE;
    // }
    // f["cam_fps"] >> fps;
	// f.release();

    // //根据敌方颜色初始化相机模块
    // robot_cam_uestc::MindVisionCamDev cam_intercace;
    // if(cam_intercace.init(config_path) != 0)
    //     return EXIT_FAILURE;
    
    // //设置相机公有化参数
    // cam_intercace.setParameter(robot_cam::Fps, fps);
  
    // robot_cam_uestc::RobotCamThread cam_thread;
    // cam_thread.init(&cam_intercace, config_path);

    // //img read and process
    // Mat img;
    // auto cap_time = chrono::system_clock::now();//record time
    // while (true) {
    //     if (cam_thread.capImg(img) == 0) {
    //         double fps_cap_ms =
    //                 chrono::duration_cast<chrono::duration<double> >  \
    //                 (chrono::system_clock::now() -cap_time).count() * 1000;
    //         cap_time = chrono::system_clock::now();
    //         cout<<"fps:"<<1000/fps_cap_ms<<endl;
    //         imshow("img", img);
    //         char key=cv::waitKey(1);
    //         if(key=='b'){
    //             cam_thread.setResolution(1280,720);
    //         }else if(key=='s'){
    //             cam_thread.setResolution(960,600);
    //         }else if(key=='p'){
    //             int value;
    //             if(cam_thread.getParameter(robot_cam::ResolutionHeight,value)){
    //                 cout<<"height"<<value<<endl;
    //             }
    //         }else if(key=='q'){
    //             break;
    //         }
    //         //std::this_thread::sleep_for(std::chrono::milliseconds(10));//algo time

    //     }
    // }
    
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
  
    ros::waitForShutdown();
    return EXIT_SUCCESS;
}