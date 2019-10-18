/****************************************************************************
 *  Copyright (C) 2019 RobotOS.
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
 *  file  : image_tool.h
 *  brief : 图像处理工具类
 *  author: FWC
 *  date  : 2019-2-13
 ***************************************************************************/
#include "task_image_related/image_tool.h"
using namespace std;
using namespace cv;
using namespace task_image_related;


bool ImageTool::is_debug=false;

float ImageTool::calc2PointDistance(cv::Point2f point1, cv::Point2f point2) {
    return sqrt(((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)));
}

float ImageTool::calc2PointDistance(cv::Point3f point1, cv::Point3f point2) {
    return sqrt((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)+(point1.z-point2.z)*(point1.z-point2.z));
}

//0-180,90为垂直
float ImageTool::calc2PointAngle(cv::Point2f point1, cv::Point2f point2) {
    float angle;
    if(point1.x == point2.x){
        return 90;
    } else{
        double k;
        k= -(point1.y - point2.y) / (point1.x - point2.x);//符号取反，图像坐标系和实际坐标系不统一
        angle=(float)(atan(k)*180/CV_PI);
    }
    if(angle<0){
         angle=angle+180;
    }
     return angle;

}

float ImageTool::calcTriangleInnerAngle(cv::Point2f vertexPoint, cv::Point2f point1, cv::Point2f point2) {
    float a,b,c;//求角C
    float angleC;
    a=calc2PointDistance(vertexPoint,point1);
    b=calc2PointDistance(vertexPoint,point2);
    c=calc2PointDistance(point1,point2);
    angleC=static_cast<float>(acos((a * a + b * b - c * c) / (2 * a * b)) / CV_PI * 180);
    return angleC;

}

//调试部分
//绘制旋转矩形
Scalar ImageTool::getColor(int type){
     Scalar color;
    if(type==0){
        color = Scalar(255,0,0);
    }else if(type==1){
        color = Scalar(0,255,0);
    }else{
        color = Scalar(0,0,255);
    }
    return  color;
}


//绘制旋转矩形
void ImageTool::drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type){
    if(is_debug){
        Scalar color=getColor(type);
        Point2f rect_points[4];
        r.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( img, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
}
//绘制四边形
void ImageTool::draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type) {
    if(is_debug){
        Scalar color=getColor(type);
        for( int j = 0; j < 4; j++ )
            line( img, point2fs[j], point2fs[(j+1)%4], color, 1, 8 );
    }
}
//绘制多边形
void ImageTool::drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type){
    if(is_debug){
        Scalar color=getColor(type);
        for( int j = 0; j < points.size()-1; j++ )
            line( img, points[j], points[j+1], color, 1, 8 );
        line( img, points[0], points[points.size()-1], color, 1, 8 );
    }
}

