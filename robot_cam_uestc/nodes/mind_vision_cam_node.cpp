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
 *  file  : mind_visin_cam_node.cpp
 *  brief : MindVision工业相机模拟节点
 *  author: wyx
 *  email : 1418555317@qq.com
 *  date  : 2019-4-11
 * 
 * history:
 ***************************************************************************/
#include "ros/ros.h"
#include "robot_cam_uestc/mind_vision_cam_dev.h"
#include "robot_cam/robot_cam_example.h"
#include "robot_tool/time_tool.h"
#include <time.h>
#include <signal.h>
#include "ros/package.h"
#include "opencv2/dnn.hpp"
using namespace cv;
using namespace std;
using namespace dnn;

vector<String> getOutputsNames(const Net& net)
{
	static vector<String> names;
	if (names.empty())
	{
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		vector<int> outLayers = net.getUnconnectedOutLayers();
 
		//get the names of all the layers in the network
		vector<String> layersNames = net.getLayerNames();
 
		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}


int main(int argc, char **argv){

tm tm_; 
time_t t_; 
char buf[128]= {0}; 


    // std::string modelConfiguration = ros::package::getPath("robot_cam_uestc") + "/res/yolov3-tiny.cfg";
    // std::string modelWeights = ros::package::getPath("robot_cam_uestc") + "/res/yolov3-tiny_500.weights";
    // Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    
    // net.setPreferableBackend(3);
    // net.setPreferableTarget(1);

    std::string config_path = ros::package::getPath("robot_cam_uestc") + "/res/mind_vision_cam_config.yaml";
    std::string save_pic_path = ros::package::getPath("robot_cam_uestc") + "/save_pic/";
    //根据敌方颜色初始化相机模块
    robot_cam_uestc::MindVisionWithoutThread cam_intercace;
    if(cam_intercace.init(960,600,config_path) != 0)
        return EXIT_FAILURE;
    //设置相机公有化参数
    //cam_intercace.setParameter(robot_cam::Fps, fps);
    //cam_intercace.readParameters(); 
    // robot_cam::RobotCamExample cam_example;
    // cam_example.init(&cam_intercace, config_path);
    Mat src;
    bool movie_cap = false;
    // vector<Mat> outs;
    robot_tool::TimeTool timetool;
    int pic_count = 0;
    VideoWriter writer;
    writer.open(save_pic_path+"movie.avi",CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(960,600), true);
    while(true){
        time(&t_);
        tm_ = *localtime(&t_);
        strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &tm_); 

        int start = timetool.currentTimeMsGet();

        cout<<"t is:"<<buf+to_string(start)<<endl;
        cam_intercace.getImg(src);
        // net.setInput(src);
        // net.forward(outs, getOutputsNames(net));
        int end = timetool.currentTimeMsGet();
        cout<<"time cost is: "<<(end-start)<<" size: "<<src.size()<<"\n";
        imshow("Src",src);
        if(movie_cap){
            writer.write(src);
        }
        char key=cv::waitKey(1);
        if(key=='o'){

            imwrite(save_pic_path + buf+to_string(start)+".jpg",src);
            pic_count++;
        }
        if(key == 'p'){
            movie_cap = true;
        }
        cam_intercace.adjustParams(key);
    }
  
    //ros::waitForShutdown();
    return EXIT_SUCCESS;
}

