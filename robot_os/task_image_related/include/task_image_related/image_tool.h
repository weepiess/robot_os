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
 *  author: gezp
 *  email : 1350824033@qq.com
 *  date  : 2019-2-13
 ***************************************************************************/
#include "opencv2/opencv.hpp"

#ifndef TASK_IAMGE_RELATED_IMAGE_TOOL_H
#define TASK_IAMGE_RELATED_IMAGE_TOOL_H

#define IMAGE_TOOL_DEBUG(text) \
    if (task_image_related::ImageTool::is_debug) text
      

namespace task_image_related {

class ImageTool{
public:

    static float calc2PointDistance(cv::Point2f point1,cv::Point2f point2);
    static float calc2PointDistance(cv::Point3f point1,cv::Point3f point2);
    static float calc2PointAngle(cv::Point2f point1,cv::Point2f point2);
    static float calcTriangleInnerAngle(cv::Point2f vertexPoint,cv::Point2f point1,cv::Point2f point2);
    //
    static bool is_debug;
    //
    static cv::Scalar getColor(int type);
    static void drawRotatedRect(cv::Mat &img,cv::RotatedRect r,int type=0);
    static void draw4Point4f(cv::Mat &img, cv::Point2f *point2fs,int type=0);
    static void drawConvexHull(cv::Mat &img,std::vector<cv::Point2f> points,int type=0);
    

};

}

#endif //TASK_IAMGE_RELATED_IMAGE_TOOL_H

