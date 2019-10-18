////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Tool Code for robot
///ALL RIGHTS RESERVED
///@file:basic_tool.h
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-13
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#ifndef BASIC_TOOL_H
#define BASIC_TOOL_H

#include "opencv2/opencv.hpp"
#include "robot_tool/pnp_solver.h"
#include <functional>
#include <time.h>

namespace sentry_tool_uestc{

    class BasicTool{
    public:
        /** 读入相机参数
         * @param: filename, 参数路径
         * @param: pnp_solver, PNPSolver实例
         * @note: 要求传入的文件路径名为.yaml后缀，并且相机内参和畸变参数名为fx,fy,cx,cy,rdx,rdy,tdx,tdy
         */
        static void readCameraParams(const std::string& file_name, robot_tool::PNPSolver& pnp_solver);

        static void readCameraParams(const std::string& file_name, std::vector<robot_tool::PNPSolver*>& pnp_solvers);

        /** 计算相机目标到相机的垂直距离，相机坐标系
         * @param: y, z, 解算出的目标距离加上需要的偏置
         * @param: curr_pitch, 当前pitch角度
         * @return: float, 垂直距离
         */ 
        static float calTargetYDis(double y, double z, float curr_pitch);

        /** 获取指定段中的一点随机位置
         * @param: min_pos, max_pos, 起始点和终止点
         * @return: int, 随机位置
         */
        static int getRandomPos(int min_pos, int max_pos);
    };

};

#endif //SENTRYDEMO_BASIC_TOOL_H
