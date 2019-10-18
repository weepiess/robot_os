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
 *  file  : mono_measure_tool.h
 *  brief : 基于单目视觉位置测量。
 *  author: FWC
 *  date  : 2018-12-03
 * history:
 * 2019-4-4:add code annotation
 ***************************************************************************/
#ifndef TASK_IMAGE_RELATED_MONO_MEASURE_TOOL_H
#define TASK_IMAGE_RELATED_MONO_MEASURE_TOOL_H
#include <iostream>
#include "opencv2/opencv.hpp"

namespace task_image_related {

class MonoMeasureTool {
   private:
    cv::Mat intrinsic_matrix_;   //相机内参
    cv::Mat distortion_coeffs_;  //相机外参
    //测量结果
    cv::Point3f position_;

   public:
    int init(std::string filename);
    //////////3d点坐标求解（use solve pnp）
    // points2d: input,一组图像上的2d点（4个点）
    // points3d: input,一组3d点（世界坐标系），对应图像上的点（4个点）
	// position: output,世界坐标系原点在相机坐标系下的位置。
	// return :state
    int solvePnP4Points(std::vector<cv::Point2f>& points2d,
                        std::vector<cv::Point3f>& points3d,
                        cv::Point3f& position);
	//////3d点坐标求解
	//p: intput,图像上点坐标
	//distance: input,已知的真实距离
	//return :对应的真实3d点坐标 
    cv::Point3f imagePoint2CameraFrame(cv::Point2f p, double distance);
	//////视角求解
	//p: intput,图像上点坐标
    //pitch: output,视角pitch
	//yaw: output,视角yaw
	//return :state 
    int imagePoint2ViewAngle(cv::Point2f p, float& pitch, float& yaw);
};

}  // namespace task_image_related
#endif  // TASK_IMAGE_RELATED_MONO_MEASURE_TOOL_H
