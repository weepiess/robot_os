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
 *  file  : robot_cam_thread_example.h
 *  brief : usb相机多线程测试例程
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-1-30
 ***************************************************************************/
#include <opencv2/opencv.hpp>
#include "robot_cam_uestc/robot_cam_thread.h"
#include "robot_cam/usb_cam_dev.h"
#include "ros/package.h"
#include "ros/ros.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "usb_cam_thread_node",
              ros::init_options::NoSigintHandler);
    std::string config_path =
        ros::package::getPath("robot_cam") + "/res/usb_cam_config.yaml";
    //usb cam dev
    robot_cam::UsbCamDev cam_intercace;
    cam_intercace.init(config_path);
    //cam thread
    robot_cam_uestc::RobotCamThread cam_thread;
    cam_thread.init(&cam_intercace, config_path);
    //img read and process
    Mat img;
    auto cap_time = chrono::system_clock::now();//record time
    while (true) {
        if (cam_thread.capImg(img) == 0) {
            double fps_cap_ms =
                    chrono::duration_cast<chrono::duration<double> >  \
                    (chrono::system_clock::now() -cap_time).count() * 1000;
            cap_time = chrono::system_clock::now();
            cout<<"fps:"<<1000/fps_cap_ms<<endl;
            imshow("img", img);
            char key=cv::waitKey(1);
            if(key=='b'){
                cam_thread.setResolution(1280,720);
            }else if(key=='s'){
                cam_thread.setResolution(640,480);
            }else if(key=='p'){
                int value;
                if(cam_thread.getParameter(robot_cam::ResolutionHeight,value)){
                    cout<<"height"<<value<<endl;
                }
            }else if(key=='q'){
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));//algo time

        }
    }
    ros::waitForShutdown();
    return 0;
}
