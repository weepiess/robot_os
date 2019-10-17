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
 *  file  : task_show_image.cpp
 *  brief : 显示图像任务
 *  author: FWC
 *  date  : 2019-2-13
 ****************************************************************************/

#include "task_image_related/task_show_image.h"

using namespace cv;
using namespace std;
using namespace task_image_related;

TaskShowImage::TaskShowImage(){

}

TaskShowImage::~TaskShowImage(){

}

int TaskShowImage::initTask(std::string conf_path){
    setRunFlag(true);
    cout<<"task show image init"<<endl;
    return 0;
}

void TaskShowImage::taskImageProcess(cv::Mat& img,double img_stamp){
    imshow("show_img",img);
    waitKey(1);
    cout<<"task show image,get image"<<endl;
}
void TaskShowImage::taskImageWait(){
    cout<<"task show image,wait get image"<<endl;
}
void TaskShowImage::taskSleep(){
    
    cout<<"task show image,sleep......"<<endl;
}