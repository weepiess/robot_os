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
 *  file  : mono_measure_tool.cpp
 *  brief : 基于单目视觉位置测量。
 *  author: FWC
 *  date  : 2018-12-03
 ***************************************************************************/
#include "task_image_related/mono_measure_tool.h"

using namespace std;
using namespace cv;
using namespace task_image_related; 

int MonoMeasureTool::init(string filename)
{
	FileStorage f(filename, FileStorage::READ);
        if (!f.isOpened())
        {
            cerr << "Failed to open " << filename << endl;
            return 1;
        }
	f["intrinsic_matrix"] >> intrinsic_matrix_;
	f["distortion_coeffs"] >> distortion_coeffs_;
	f.release();
}

int MonoMeasureTool::solvePnP4Points(vector<Point2f>& points2d,vector<Point3f>& points3d,cv::Point3f &position)
{	
        if(points2d.size()!=points3d.size()){
              return 1;//投影点数量不匹配
        }
	Mat rot = Mat::eye(3, 3, CV_64FC1);
	Mat trans = Mat::zeros(3, 1, CV_64FC1);
	Mat r; //旋转向量
	solvePnP(points3d,points2d,intrinsic_matrix_,distortion_coeffs_,r,trans);
	position = Point3f(trans);
        return 0;
        
}

//refer to :http://www.cnblogs.com/singlex/p/pose_estimation_1_1.html
//根据输入的参数将图像坐标转换到相机坐标中
//输入为图像上的点坐标
//double distance 物距
//输出3d点坐标的单位与distance（物距）的单位保持一致
cv::Point3f MonoMeasureTool::imagePoint2CameraFrame(cv::Point2f p, double distance)
{
    double fx;
    double fy;
    double u0;
    double v0;
 
    fx = intrinsic_matrix_.ptr<double>(0)[0];
    u0 = intrinsic_matrix_.ptr<double>(0)[2];
    fy = intrinsic_matrix_.ptr<double>(1)[1];
    v0 = intrinsic_matrix_.ptr<double>(1)[2];
    double zc = distance;
    double xc = (p.x - u0)*distance / fx;
    double yc = (p.y - v0)*distance / fy;
    return cv::Point3f(xc, yc, zc);
}

//获取image任意点的视角，pitch，yaw（相对相机坐标系）。
//与相机坐标系保持一致。
int MonoMeasureTool::imagePoint2ViewAngle(cv::Point2f p,float& pitch,float& yaw){
    double fx;
    double fy;
    double u0;
    double v0;
 
    fx = intrinsic_matrix_.ptr<double>(0)[0];
    u0 = intrinsic_matrix_.ptr<double>(0)[2];
    fy = intrinsic_matrix_.ptr<double>(1)[1];
    v0 = intrinsic_matrix_.ptr<double>(1)[2];

    pitch=atan2((p.y - v0),fy);
    yaw=atan2((p.x - u0),fx);
    return 0;

}